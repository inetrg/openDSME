/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * described in the IEEE 802.15.4-2015 standard
 *
 * Authors: Florian Kauer <florian.kauer@tuhh.de>
 *          Maximilian Koestler <maximilian.koestler@tuhh.de>
 *          Sandrina Backhauss <sandrina.backhauss@tuhh.de>
 *
 * Based on
 *          DSME Implementation for the INET Framework
 *          Tobias Luebkert <tobias.luebkert@tuhh.de>
 *
 * Copyright (c) 2015, Institute of Telematics, Hamburg University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "./MessageDispatcher.h"

#include "opendsme/dsme_platform.h"
#include "opendsme/dsme_settings.h"
#include "../../helper/DSMEDelegate.h"
#include "../../helper/Integers.h"
#include "../../interfaces/IDSMEMessage.h"
#include "../../interfaces/IDSMEPlatform.h"
#include "../../mac_services/DSME_Common.h"
#include "../../mac_services/MacDataStructures.h"
#include "../../mac_services/dataStructures/DSMEAllocationCounterTable.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/mcps_sap/DATA.h"
#include "../../mac_services/mcps_sap/MCPS_SAP.h"
#include "../../mac_services/pib/dsme_mac_constants.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../../mac_services/pib/PHY_PIB.h"
#include "../../mac_services/pib/PIBHelper.h"
#include "../DSMELayer.h"
#include "../ackLayer/AckLayer.h"
#include "../associationManager/AssociationManager.h"
#include "../beaconManager/BeaconManager.h"
#include "../capLayer/CAPLayer.h"
#include "../gtsManager/GTSManager.h"
#include "../messages/IEEE802154eMACHeader.h"
#include "../messages/MACCommand.h"

namespace dsme {

MessageDispatcher::MessageDispatcher(DSMELayer& dsme)
    : dsme(dsme),
      currentACTElement(nullptr, nullptr),
      doneGTS(DELEGATE(&MessageDispatcher::sendDoneGTS, *this)),
      dsmeAckFrame(nullptr),
      lastSendGTSNeighbor(neighborQueue.end()) {
}

MessageDispatcher::~MessageDispatcher() {
    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
        while(!this->neighborQueue.isQueueEmpty(it)) {
            IDSMEMessage* msg = neighborQueue.popFront(it);
            this->dsme.getPlatform().releaseMessage(msg);
        }
    }
}

void MessageDispatcher::initialize(void) {
    currentACTElement = dsme.getMAC_PIB().macDSMEACT.end();
    return;
}

void MessageDispatcher::reset(void) {
    currentACTElement = dsme.getMAC_PIB().macDSMEACT.end();

    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
        while(!this->neighborQueue.isQueueEmpty(it)) {
            IDSMEMessage* msg = neighborQueue.popFront(it);
            mcps_sap::DATA_confirm_parameters params;
            params.msduHandle = msg;
            params.timestamp = 0;
            params.rangingReceived = false;
            params.gtsTX = true;
            params.status = DataStatus::TRANSACTION_EXPIRED;
            params.numBackoffs = 0;
            this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
        }
    }
    while(this->neighborQueue.getNumNeighbors() > 0) {
        NeighborQueue<MAX_NEIGHBORS>::iterator it = this->neighborQueue.begin();
        this->neighborQueue.eraseNeighbor(it);
    }

    return;
}


void MessageDispatcher::sendDoneGTS(enum AckLayerResponse response, IDSMEMessage* msg) {
    LOG_DEBUG("sendDoneGTS");

    DSME_ASSERT(lastSendGTSNeighbor != neighborQueue.end());
    DSME_ASSERT(msg == neighborQueue.front(lastSendGTSNeighbor));

    DSMEAllocationCounterTable& act = this->dsme.getMAC_PIB().macDSMEACT;
    DSME_ASSERT(this->currentACTElement != act.end());

    if(this->multiplePacketsPerGTS) {
        //TODO: setting up the IFS timer only makes sense when there actually is
        //      enough time left for another transmission in the current slot
        this->dsme.getEventDispatcher().setupIFSTimer(msg->getTotalSymbols() > aMaxSIFSFrameSize);
    }

    if(response != AckLayerResponse::NO_ACK_REQUESTED && response != AckLayerResponse::ACK_SUCCESSFUL) {
        currentACTElement->incrementIdleCounter();

        // not successful -> retry?
        if(msg->getRetryCounter() < dsme.getMAC_PIB().macMaxFrameRetries) {
            msg->increaseRetryCounter();
            finalizeGTSTransmission();
            LOG_DEBUG("sendDoneGTS - retry");
            return; // will stay at front of queue
        }
    }

    transceiverOffIfAssociated();

    if(response == AckLayerResponse::ACK_FAILED || response == AckLayerResponse::ACK_SUCCESSFUL) {
        this->dsme.getPlatform().signalAckedTransmissionResult(response == AckLayerResponse::ACK_SUCCESSFUL, msg->getRetryCounter() + 1, msg->getHeader().getDestAddr());
    }

    neighborQueue.popFront(lastSendGTSNeighbor);
    this->preparedMsg = nullptr;

    /* STATISTICS */
    uint16_t totalSize = 0;
    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
        totalSize += it->queueSize;
    }
    this->dsme.getPlatform().signalQueueLength(totalSize);
    /* END STATISTICS */

    mcps_sap::DATA_confirm_parameters params;
    params.msduHandle = msg;
    params.timestamp = 0; // TODO
    params.rangingReceived = false;
    params.gtsTX = true;

    switch(response) {
        case AckLayerResponse::NO_ACK_REQUESTED:
        case AckLayerResponse::ACK_SUCCESSFUL:
            LOG_DEBUG("sendDoneGTS - success");
            params.status = DataStatus::SUCCESS;
            break;
        case AckLayerResponse::ACK_FAILED:
            DSME_ASSERT(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end());
            currentACTElement->incrementIdleCounter();
            params.status = DataStatus::NO_ACK;
            break;
        case AckLayerResponse::SEND_FAILED:
            LOG_DEBUG("SEND_FAILED during GTS");
            DSME_ASSERT(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end());
            currentACTElement->incrementIdleCounter();
            params.status = DataStatus::CHANNEL_ACCESS_FAILURE;
            break;
        case AckLayerResponse::SEND_ABORTED:
            LOG_DEBUG("SEND_ABORTED during GTS");
            params.status = DataStatus::TRANSACTION_EXPIRED;
            break;
        default:
            DSME_ASSERT(false);
    }

    params.numBackoffs = 0;
    this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);


    if(!this->multiplePacketsPerGTS || !prepareNextMessageIfAny()) {
        /* '-> prepare next frame for transmission after one IFS */
        finalizeGTSTransmission();
    }
}

void MessageDispatcher::finalizeGTSTransmission() {
    LOG_DEBUG("Finalizing transmission for " << this->currentACTElement->getGTSlotID() << " " << this->currentACTElement->getSuperframeID() << " " << this->currentACTElement->getChannel());
    if(this->multiplePacketsPerGTS) {
        this->dsme.getEventDispatcher().stopIFSTimer();
    }
    this->preparedMsg = nullptr;    // TODO correct here?
    this->lastSendGTSNeighbor = this->neighborQueue.end();
    this->currentACTElement = this->dsme.getMAC_PIB().macDSMEACT.end();
    if(this->numTxGtsFrames > 0) this->dsme.getPlatform().signalPacketsTXPerSlot(this->numTxGtsFrames);
    if(this->numRxGtsFrames > 0) this->dsme.getPlatform().signalPacketsRXPerSlot(this->numRxGtsFrames);
    this->numTxGtsFrames = 0;
    this->numRxGtsFrames = 0;
}

void MessageDispatcher::onCSMASent(IDSMEMessage* msg, DataStatus::Data_Status status, uint8_t numBackoffs, uint8_t transmissionAttempts) {
    if(status == DataStatus::Data_Status::NO_ACK || status == DataStatus::Data_Status::SUCCESS) {
        if(msg->getHeader().isAckRequested() && !msg->getHeader().getDestAddr().isBroadcast()) {
            this->dsme.getPlatform().signalAckedTransmissionResult(status == DataStatus::Data_Status::SUCCESS, transmissionAttempts,
                                                                   msg->getHeader().getDestAddr());
        }
    }

    if(msg->getReceivedViaMCPS()) {
        mcps_sap::DATA_confirm_parameters params;
        params.msduHandle = msg;
        params.timestamp = 0; // TODO
        params.rangingReceived = false;
        params.status = status;
        params.numBackoffs = numBackoffs;
        params.gtsTX = false;
        this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
    } else {
        if(msg->getHeader().getFrameType() == IEEE802154eMACHeader::FrameType::COMMAND) {
            MACCommand cmd;
            cmd.decapsulateFrom(msg);

            LOG_DEBUG("cmdID " << (uint16_t)cmd.getCmdId());

            switch(cmd.getCmdId()) {
                case ASSOCIATION_REQUEST:
                case ASSOCIATION_RESPONSE:
                case DISASSOCIATION_NOTIFICATION:
                    this->dsme.getAssociationManager().onCSMASent(msg, cmd.getCmdId(), status, numBackoffs);
                    break;
                case DATA_REQUEST:
                case DSME_ASSOCIATION_REQUEST:
                case DSME_ASSOCIATION_RESPONSE:
                    DSME_ASSERT(false);
                    // TODO handle correctly
                    this->dsme.getPlatform().releaseMessage(msg);
                    break;
                case BEACON_REQUEST:
                case DSME_BEACON_ALLOCATION_NOTIFICATION:
                case DSME_BEACON_COLLISION_NOTIFICATION:
                    this->dsme.getBeaconManager().onCSMASent(msg, cmd.getCmdId(), status, numBackoffs);
                    break;
                case DSME_GTS_REQUEST:
                case DSME_GTS_REPLY:
                case DSME_GTS_NOTIFY:
                    this->dsme.getGTSManager().onCSMASent(msg, cmd.getCmdId(), status, numBackoffs);
                    break;
            }
        } else {
            this->dsme.getPlatform().releaseMessage(msg);
        }
    }
}

bool MessageDispatcher::sendInGTS(IDSMEMessage* msg, NeighborQueue<MAX_NEIGHBORS>::iterator destIt) {
    DSME_ASSERT(!msg->getHeader().getDestAddr().isBroadcast());
    DSME_ASSERT(this->dsme.getMAC_PIB().macAssociatedPANCoord);
    DSME_ASSERT(destIt != neighborQueue.end());

    numUpperPacketsForGTS++;

    if(!neighborQueue.isQueueFull()) {
        /* push into queue */
        // TODO implement TRANSACTION_EXPIRED
        uint16_t totalSize = 0;
        for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
            totalSize += it->queueSize;
        }
        LOG_INFO("NeighborQueue is at " << totalSize << "/" << TOTAL_GTS_QUEUE_SIZE << ".");
        neighborQueue.pushBack(destIt, msg);
        this->dsme.getPlatform().signalQueueLength(totalSize+1);
        return true;
    } else {
        /* queue full */
        LOG_INFO("NeighborQueue is full!");
        numUpperPacketsDroppedFullQueue++;
        return false;
    }
}

bool MessageDispatcher::sendInCAP(IDSMEMessage* msg) {
    LOG_INFO("Inserting message into CAP queue.");
    if(msg->getHeader().getSrcAddrMode() != EXTENDED_ADDRESS && !(this->dsme.getMAC_PIB().macAssociatedPANCoord)) {
        LOG_INFO("Message dropped due to missing association!");
        // TODO document this behaviour
        // TODO send appropriate MCPS confirm or better remove this handling and implement TRANSACTION_EXPIRED
        return false;
    }
    if(!this->dsme.getCapLayer().pushMessage(msg)) {
        LOG_INFO("CAP queue full!");
        return false;
    }
    if(msg->getHeader().getFrameType() == IEEE802154eMACHeader::FrameType::COMMAND) {
        numUpperPacketsForCAP++;
    }
    return true;
}

void MessageDispatcher::onReceive(IDSMEMessage* msg) {
    IEEE802154eMACHeader macHdr = msg->getHeader();

    switch(macHdr.getFrameType()) {
        case IEEE802154eMACHeader::FrameType::DATA: {
            if(currentACTElement != dsme.getMAC_PIB().macDSMEACT.end()) {
                handleGTSFrame(msg);
            }
            break;
        }

        default: {
            LOG_ERROR((uint16_t)macHdr.getFrameType());
            dsme.getPlatform().releaseMessage(msg);
        }
    }
    return;
}

bool MessageDispatcher::handlePreSlotEvent(uint8_t nextSlot, uint8_t nextSuperframe, uint8_t nextMultiSuperframe) {
    // Prepare next slot
    // Switch to next slot channel and radio mode
    DSMEAllocationCounterTable& act = this->dsme.getMAC_PIB().macDSMEACT;
    if(this->currentACTElement != act.end()) {
        if(this->currentACTElement->getDirection() == Direction::RX) {
            this->currentACTElement = act.end();
        } else {
            // Rarely happens, only if the sendDoneGTS is delayed
            // Then skip this preSlotEvent
            LOG_DEBUG("Previous slot did not finish until preslot event: slot " << (int)nextSlot << " SF " << (int)nextSuperframe);
            DSME_SIM_ASSERT(false);
            return false;
        }
    }

    if(nextSlot > this->dsme.getMAC_PIB().helper.getFinalCAPSlot(nextSuperframe)) {
        /* '-> next slot will be GTS */
        if (this->dsme.getMAC_PIB().macIsPANCoord || this->dsme.getBeaconManager().isSynced()) {
            if(this->dsme.getMAC_PIB().macIsPANCoord) {
                DBG_PIN_CLEAR(LA_PIN_COORD_UPDT_DLGT_BCN_CAP_CFP);
            } else {
                DBG_PIN_CLEAR(LA_PIN_RFD_UPDT_DLGT_BCN_CAP_CFP);
            }
            this->dsme.getPlatform().setReceiveDelegate(DELEGATE(&MessageDispatcher::onReceive, *this));
        }

        unsigned nextGTS = nextSlot - (this->dsme.getMAC_PIB().helper.getFinalCAPSlot(nextSuperframe) + 1);
        if(act.isAllocated(nextSuperframe, nextGTS)) {
            /* '-> this slot might be used */

            this->currentACTElement = act.find(nextSuperframe, nextGTS);
            DSME_ASSERT(this->currentACTElement != act.end());
            // For TX currentACTElement will be reset in finalizeGTSTransmission, called by
            // either handleGTS if nothing is to send or by sendDoneGTS.
            // For RX it is reset in the next handlePreSlotEvent.   TODO: is the reset actually required?

            // For RX also if INVALID or UNCONFIRMED!
            if((this->currentACTElement->getState() == VALID) || (this->currentACTElement->getDirection() == Direction::RX)) {
                this->dsme.getPlatform().turnTransceiverOn();

                if(dsme.getMAC_PIB().macChannelDiversityMode == Channel_Diversity_Mode::CHANNEL_ADAPTATION) {
                    this->dsme.getPlatform().setChannelNumber(this->dsme.getMAC_PIB().helper.getChannels()[this->currentACTElement->getChannel()]);
                } else {
                    uint8_t channel = nextHoppingSequenceChannel(nextSlot, nextSuperframe, nextMultiSuperframe);
                    this->dsme.getPlatform().setChannelNumber(channel);
                }

                if (this->currentACTElement->getDirection() == Direction::RX) {
                    this->dsme.getPlatform().turnTransceiverToRX();
                    // statistic - gets decremented on actual reception
                    this->numUnusedRxGts++;
                }
            }

            if((this->currentACTElement->getState() == VALID) && (this->currentACTElement->getDirection() == Direction::TX)) {
                if (prepareNextMessageIfAny()) {
                    this->msg_preloaded_already = this->dsme.getAckLayer().prepareSendingCopy(this->preparedMsg, this->doneGTS);
                } else {
                    this->dsme.getPlatform().turnTransceiverToIdle();
                }
            }
        } else {
            /* '-> nothing to do during this slot */
            DSME_ASSERT(this->currentACTElement == act.end());
            transceiverOffIfAssociated();
        }
    } else if(nextSlot == 0) {
        /* '-> beacon slots are handled by the BeaconManager */
        DSME_ASSERT(this->currentACTElement == act.end());
    } else if(nextSlot == 1) {
        /* '-> next slot will be CAP */
        if((!this->dsme.getMAC_PIB().macCapReduction || nextSuperframe == 0)
                && this->dsme.getPlatform().isRxEnabledOnCap()) {
            /* '-> active CAP slot */
            if (this->dsme.getMAC_PIB().macIsPANCoord || this->dsme.getBeaconManager().isSynced()) {
                if(this->dsme.getMAC_PIB().macIsPANCoord) {
                    DBG_PIN_SET(LA_PIN_COORD_UPDT_DLGT_BCN_CAP_CFP);
                } else {
                    DBG_PIN_SET(LA_PIN_RFD_UPDT_DLGT_BCN_CAP_CFP);
                }
                this->dsme.getPlatform().setReceiveDelegate(DELEGATE(&CAPLayer::onReceive, this->dsme.getCapLayer()));
            }
            this->dsme.getPlatform().turnTransceiverOn();
            this->dsme.getPlatform().setChannelNumber(this->dsme.getPHY_PIB().phyCurrentChannel);
            this->dsme.getPlatform().turnTransceiverToRX();
        } else {
            /* '-> CAP reduction */
            if (this->dsme.getMAC_PIB().macIsPANCoord || this->dsme.getBeaconManager().isSynced()) {
                this->dsme.getPlatform().setReceiveDelegate(DELEGATE(&MessageDispatcher::onReceive, *this));
            }
            transceiverOffIfAssociated();
        }
    }

    return true;
}

uint8_t MessageDispatcher::nextHoppingSequenceChannel(uint8_t nextSlot, uint8_t nextSuperframe, uint8_t nextMultiSuperframe) {
    uint16_t hoppingSequenceLength = this->dsme.getMAC_PIB().macHoppingSequenceLength;
    uint8_t ebsn = this->dsme.getMAC_PIB().macPanCoordinatorBsn;
    uint16_t sdIndex = nextSuperframe + this->dsme.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe() * nextMultiSuperframe;
    uint8_t numGTSlots = this->dsme.getMAC_PIB().helper.getNumGTSlots(sdIndex);

    uint8_t slotId = this->currentACTElement->getGTSlotID();
    uint16_t channelOffset = this->currentACTElement->getChannel();

    uint8_t channel =
        this->dsme.getMAC_PIB().macHoppingSequenceList[(sdIndex * numGTSlots + slotId + channelOffset + ebsn) % hoppingSequenceLength];
    LOG_INFO("Using channel " << channel << " - numGTSlots: " << numGTSlots << " EBSN: " << ebsn << " sdIndex: " << sdIndex
                              << " slot: " << slotId << " Superframe " << nextSuperframe << " channelOffset: " << channelOffset
                              << " Direction: " << currentACTElement->getDirection());
    return channel;
}

bool MessageDispatcher::handleSlotEvent(uint8_t slot, uint8_t superframe, int32_t lateness) {
    if(slot > dsme.getMAC_PIB().helper.getFinalCAPSlot(superframe)) {
        handleGTS(lateness);
    }
    return true;
}

bool MessageDispatcher::handleIFSEvent(int32_t lateness) {
    /* this callback shall only be called if the (non-standard) feature for
     * multiple packet transmissions per GTS is enabled */
    assert(this->multiplePacketsPerGTS);
    /* Neighbor and slot have to be valid at this point */
    DSME_ASSERT(this->lastSendGTSNeighbor != this->neighborQueue.end());
    DSME_ASSERT(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end());
    DSME_ASSERT(this->currentACTElement->getSuperframeID() == this->dsme.getCurrentSuperframe());
    uint16_t gtsid = this->currentACTElement->getGTSlotID();
    uint16_t csid = this->dsme.getCurrentSlot();
    uint16_t fcsid = this->dsme.getMAC_PIB().helper.getFinalCAPSlot(this->dsme.getCurrentSuperframe());
    if (gtsid != (csid - (fcsid+1))) {
        assert(false);
    }

    if(!sendPreparedMessage()) {
        finalizeGTSTransmission();
    }

    return true;
}


void MessageDispatcher::handleGTS(int32_t lateness) {

    if(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end() && this->currentACTElement->getSuperframeID() == this->dsme.getCurrentSuperframe() &&
       this->currentACTElement->getGTSlotID() ==
           this->dsme.getCurrentSlot() - (this->dsme.getMAC_PIB().helper.getFinalCAPSlot(dsme.getCurrentSuperframe()) + 1)) {
        /* '-> this slot matches the prepared ACT element */
        if(this->currentACTElement->getDirection() == RX) { // also if INVALID or UNCONFIRMED!
            /* '-> a message may be received during this slot */

        } else if(this->currentACTElement->getState() == VALID) {
            /* '-> if any messages are queued for this link, send one */

            bool success = this->msg_preloaded_already || prepareNextMessageIfAny();

            LOG_DEBUG(success);
            if(success) {
                /* '-> a message is queued for transmission */
                success = sendPreparedMessage();
            }
            LOG_DEBUG(success);

            if(success == false) {
                /* '-> no message to be sent */
                LOG_DEBUG("MessageDispatcher: Could not transmit any packet in GTS");
                this->numUnusedTxGts++;
                finalizeGTSTransmission();
            }
        } else {
            finalizeGTSTransmission();
        }
    }
}
void MessageDispatcher::handleGTSFrame(IDSMEMessage* msg) {
    DSME_ASSERT(currentACTElement != dsme.getMAC_PIB().macDSMEACT.end());

    numRxGtsFrames++;
    numUnusedRxGts--;

    if(currentACTElement->getSuperframeID() == dsme.getCurrentSuperframe() &&
       currentACTElement->getGTSlotID() == dsme.getCurrentSlot() - (dsme.getMAC_PIB().helper.getFinalCAPSlot(dsme.getCurrentSuperframe()) + 1)) {
        // According to 5.1.10.5.3
        currentACTElement->resetIdleCounter();
    }

    createDataIndication(msg);
}


// Function to determine if there is any message that can be transmitted or not.
// Returns: True: 1. If there is a pending message (e.g. from failed transmission) to transmit
//                2. If there exits any message in the queue to transmit.
//          False otherwise
bool MessageDispatcher::prepareNextMessageIfAny() {
    if(this->preparedMsg) {
        // a message was prepared before
        return true;
    } else {
        IEEE802154MacAddress adr = IEEE802154MacAddress(this->currentACTElement->getAddress());
        this->lastSendGTSNeighbor = this->neighborQueue.findByAddress(IEEE802154MacAddress(this->currentACTElement->getAddress()));

        // there is no pending message, then check if the queue is empty (i.e. there is not any message to transmit to a target neighbor)
        // if there is a message to send retrieve a copy of it from the queue and set the flag to check if possible to send the message
        this->preparedMsg = this->neighborQueue.front(this->lastSendGTSNeighbor);
        if (this->preparedMsg == NULL) {
            return false;
        }
        return true;
    }
}

bool MessageDispatcher::sendPreparedMessage() {
    DSME_ASSERT(this->preparedMsg);
    uint16_t tot_syms = this->preparedMsg->getTotalSymbols();
    uint16_t awd = this->dsme.getMAC_PIB().helper.getAckWaitDuration();
    uint32_t sc = this->dsme.getPlatform().getSymbolCounter();
    DSME_ASSERT(this->dsme.getMAC_PIB().helper.getSymbolsPerSlot() >= tot_syms + awd + 10 /* arbitrary processing delay */ + PRE_EVENT_SHIFT);

    uint8_t ifsSymbols = tot_syms <= aMaxSIFSFrameSize ? const_redefines::macSIFSPeriod : const_redefines::macLIFSPeriod;
    uint32_t duration = tot_syms + awd;// + ifsSymbols ?;
    /* '-> Duration for the transmission of the next frame */
    if(this->dsme.isWithinTimeSlot(sc, duration)) {
        /* '-> Sufficient time to send message in remaining slot time */
        if (this->msg_preloaded_already) {
            /* -> Message was already loaded to the radio before.
             * Reset the preload flag if the preloaded frame was sent out
             * *and* also when the preloaded frame could not be sent at all.
             * Required as the preloaded frame will most likely be overwritten
             * during any following RX/TX. */
            this->msg_preloaded_already = false;
            this->dsme.getAckLayer().sendNowIfPending();
            this->numTxGtsFrames++;
        } else if (this->dsme.getAckLayer().prepareSendingCopy(this->preparedMsg, this->doneGTS)) {
            /* '-> Message transmission can be attempted */
            this->dsme.getAckLayer().sendNowIfPending();
            this->numTxGtsFrames++;
        } else {
            /* '-> Message could not be sent via ACKLayer (FAILED) */
            sendDoneGTS(AckLayerResponse::SEND_FAILED, this->preparedMsg);
        }
        return true;
    } else {
        /* if a message was preloaded it must be abortet here to prevent preparing another one */
        if (this->msg_preloaded_already) {
            dsme.getAckLayer().abortPreparedTransmission();
            this->msg_preloaded_already = false;
        }
    }
    LOG_DEBUG("No packet sent (remaining slot time insufficient)");
    return false;
}


void MessageDispatcher::createDataIndication(IDSMEMessage* msg) {
    IEEE802154eMACHeader& header = msg->getHeader();

    mcps_sap::DATA_indication_parameters params;

    params.msdu = msg;

    params.mpduLinkQuality = 0; // TODO link quality?
    params.dsn = header.getSequenceNumber();
    params.timestamp = msg->getStartOfFrameDelimiterSymbolCounter();
    params.securityLevel = header.isSecurityEnabled();

    params.dataRate = 0; // DSSS -> 0

    params.rangingReceived = NO_RANGING_REQUESTED;
    params.rangingCounterStart = 0;
    params.rangingCounterStop = 0;
    params.rangingTrackingInterval = 0;
    params.rangingOffset = 0;
    params.rangingFom = 0;

    this->dsme.getMCPS_SAP().getDATA().notify_indication(params);
}

void MessageDispatcher::transceiverOffIfAssociated() {
    if(this->dsme.getMAC_PIB().macAssociatedPANCoord) {
        this->dsme.getPlatform().turnTransceiverOff();
    } else {
        /* '-> do not turn off the transceiver while we might be scanning */
    }
}




} /* namespace dsme */
