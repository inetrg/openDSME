/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * described in the IEEE 802.15.4-2015 standard
 *
 * Authors: Florian Meier <florian.meier@tuhh.de>
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

#include "../dsmeLayer/DSMELayer.h" // TODO: remove cross-layer reference
#include "../mac_services/mcps_sap/MCPS_SAP.h"
#include "../mac_services/mlme_sap/MLME_SAP.h"
#include "DSMEAdaptionLayer.h"

namespace dsme {

DSMEAdaptionLayer::DSMEAdaptionLayer(DSMELayer& dsme) :
        dsme(dsme),
        associationHelper(*this),
        gtsAllocationHelper(*this),
        scanHelper(*this),

        scanInProgress(false),
        associationInProgress(false) {

}

void DSMEAdaptionLayer::initialize() {
    this->mcps_sap = &(dsme.getMCPS_SAP());
    this->mlme_sap = &(dsme.getMLME_SAP());
    this->phy_pib = &(dsme.getPHY_PIB());
    this->mac_pib = &(dsme.getMAC_PIB());

    this->associationHelper.initialize();
    this->gtsAllocationHelper.initialize();
    this->scanHelper.initialize();

    this->dsme.setStartOfCFPDelegate(DELEGATE(&GTSHelper::handleStartOfCFP, gtsAllocationHelper));
    this->mcps_sap->getDATA().indication(DELEGATE(&DSMEAdaptionLayer::handleDataIndication, *this));
    this->mcps_sap->getDATA().confirm(DELEGATE(&DSMEAdaptionLayer::handleDataConfirm, *this));
    this->scanHelper.setScanCompleteDelegate(DELEGATE(&DSMEAdaptionLayer::handleScanComplete, *this));
    this->associationHelper.setAssociationCompleteDelegate(DELEGATE(&DSMEAdaptionLayer::handleAssociationComplete, *this));
    return;
}

mcps_sap::MCPS_SAP& DSMEAdaptionLayer::getMCPS_SAP() {
    return *(this->mcps_sap);
}

mlme_sap::MLME_SAP& DSMEAdaptionLayer::getMLME_SAP() {
    return *(this->mlme_sap);
}

PHY_PIB& DSMEAdaptionLayer::getPHY_PIB() {
    return *(this->phy_pib);
}

MAC_PIB& DSMEAdaptionLayer::getMAC_PIB() {
    return *(this->mac_pib);
}

DSMELayer& DSMEAdaptionLayer::getDSME() {
    return this->dsme;
}

void DSMEAdaptionLayer::setIndicationCallback(indicationCallback_t callback_indication) {
    this->callback_indication = callback_indication;
    return;
}

void DSMEAdaptionLayer::setConfirmCallback(confirmCallback_t callback_confirm) {
    this->callback_confirm = callback_confirm;
    return;
}

void DSMEAdaptionLayer::receiveIndication(DSMEMessage* msg) {
    if (this->callback_indication) {
        callback_indication(msg);
    }
    return;
}

void DSMEAdaptionLayer::sendRetryBuffer() {
    if (this->retryBuffer.hasCurrent()) {
        DSMEAdaptionLayerBufferEntry* oldestEntry = this->retryBuffer.current();
        do {
            DSMEMessage *currentMessage = this->retryBuffer.current()->message;
            DSME_ASSERT(!currentMessage->currentlySending);

            this->retryBuffer.advanceCurrent();
            sendMessageDown(currentMessage, false);
        } while (this->retryBuffer.hasCurrent() && this->retryBuffer.current() != oldestEntry);
    }
}

void DSMEAdaptionLayer::sendMessage(DSMEMessage *msg) {
    LOG_INFO("Sending DATA message");

    sendMessageDown(msg, true);
    sendRetryBuffer();
}

void DSMEAdaptionLayer::startAssociation() {
    if (!getMAC_PIB().macAssociatedPANCoord) {
        LOG_INFO("Device is not associated with PAN.");
        if (!this->associationInProgress) {
            if (!this->scanInProgress) {
                this->scanInProgress = true;
                this->scanHelper.startScan();
            } else {
                LOG_INFO("Scan already in progress.");
            }
        } else {
            LOG_INFO("Association already in progress.");
        }
    }
}

void DSMEAdaptionLayer::sendMessageDown(DSMEMessage *msg, bool newMessage) {
    if (msg == nullptr) {
        /* '-> Error! */
        DSME_ASSERT(false);
        return;
    }
    DSME_ASSERT(!msg->currentlySending);
    msg->currentlySending = true;

    msg->getHeader().setSrcAddr(this->mac_pib->macExtendedAddress);
    IEEE802154MacAddress& dst = msg->getHeader().getDestAddr();

    if (!getMAC_PIB().macAssociatedPANCoord) {
        startAssociation();
        LOG_INFO("Discarding message for " << dst.getShortAddress() << ".");
        dsme.getPlatform().releaseMessage(msg);
        return;
    } else if (this->dsme.getDSMESettings().isCoordinator) {
        if (!associationHelper.isAssociatedDevice(dst)) {
            LOG_INFO("Discarding message for " << dst.getShortAddress() << " because it is not associated.");
            dsme.getPlatform().releaseMessage(msg);
            return;
        }
    }

    LOG_DEBUG("Sending DATA message to " << dst.getShortAddress() << " via MCPS.");

    if (dst.getShortAddress() == this->mac_pib->macShortAddress) {
        /* '-> loopback */
        LOG_INFO("Loopback message received.");
        receiveIndication(msg);
    } else {
        mcps_sap::DATA::request_parameters params;

        msg->getHeader().setSrcAddrMode(SHORT_ADDRESS);
        msg->getHeader().setDstAddrMode(SHORT_ADDRESS);
        msg->getHeader().setDstPANId(0); // TODO
        msg->getHeader().setDstAddr(dst);
        params.msdu = msg;
        params.msduHandle = 0; //TODO
        params.ackTX = true;

        if(dsme.getDSMESettings().optimizations) {
            params.gtsTX = !msg->firstTry && !dst.isBroadcast();
        }
        else {
            params.gtsTX = !dst.isBroadcast();
        }

        params.indirectTX = false;
        params.securityLevel = 0;
        params.keyIdMode = 0;
        params.keySource = nullptr;
        params.keyIndex = 0;
        params.uwbprf = PRF_OFF;
        params.ranging = NON_RANGING;
        params.uwbPreambleSymbolRepetitions = 0;
        params.dataRate = 0; // DSSS -> 0

        params.frameControlOption_pan_id_suppressed = false;
        params.frameControlOption_ies_included = false;
        params.frameControlOption_seq_num_suppressed = false;

        params.sendMultipurpose = false;

        if (params.gtsTX) {
            uint16_t srcAddr = this->dsme.getMAC_PIB().macShortAddress;
            if (srcAddr == 0xfffe) {
                LOG_WARN("No short address allocated -> cannot request GTS!");
            } else if (srcAddr == 0xffff) {
                LOG_INFO("Association required before slot allocation.");
                DSME_ASSERT(false);
            }

            if (!this->dsme.getMessageDispatcher().neighborExists(dst)) {
                // TODO implement neighbor management / routing
                this->dsme.getMessageDispatcher().addNeighbor(dst);
            }

            if(newMessage) {
                gtsAllocationHelper.indicateIncomingMessage(dst.getShortAddress());
                gtsAllocationHelper.checkAllocationForPacket(dst.getShortAddress());
            }

        } else {
            LOG_INFO("Preparing transmission in CAP.");
        }

        mcps_sap->getDATA().request(params);
    }
    return;
}

void DSMEAdaptionLayer::handleDataIndication(mcps_sap::DATA_indication_parameters &params) {
    LOG_INFO("Received DATA message from MCPS.");
    receiveIndication(params.msdu);
    return;
}

bool DSMEAdaptionLayer::queueMessageIfPossible(DSMEMessage* msg) {
    if (!this->retryBuffer.hasNext()) {
        DSME_ASSERT(this->retryBuffer.hasCurrent());

        DSMEAdaptionLayerBufferEntry* oldestEntry = this->retryBuffer.current();
        if (oldestEntry->message == msg) {
            this->retryBuffer.advanceCurrent();
            DSME_ASSERT(this->retryBuffer.hasNext());
            this->retryBuffer.next()->message = msg;
            this->retryBuffer.next()->initialSymbolCounter = this->dsme.getPlatform().getSymbolCounter();
            return true;
        }

        if (!oldestEntry->message->currentlySending) {
            uint32_t currentSymbolCounter = this->dsme.getPlatform().getSymbolCounter();
            LOG_DEBUG("DROPPED->" << oldestEntry->message->getHeader().getDestAddr().getShortAddress()
                    << ": Retry-Queue overflow ("
                    << currentSymbolCounter - oldestEntry->initialSymbolCounter
                    << " symbols old)");
            callback_confirm(oldestEntry->message, DataStatus::Data_Status::INVALID_GTS); // TODO change if queue is used for retransmissions
            this->retryBuffer.advanceCurrent();
        }
    }
    if (this->retryBuffer.hasNext()) {
        this->retryBuffer.next()->message = msg;
        this->retryBuffer.next()->initialSymbolCounter = this->dsme.getPlatform().getSymbolCounter();
        this->retryBuffer.advanceNext();
        return true; /* Do NOT release current message yet */
    }
    return false;
}

void DSMEAdaptionLayer::handleDataConfirm(mcps_sap::DATA_confirm_parameters &params) {
    LOG_DEBUG("Received DATA confirm from MCPS");
    DSMEMessage* msg = params.msduHandle;

    DSME_ASSERT(msg->currentlySending);
    msg->currentlySending = false;

    if(params.status != DataStatus::SUCCESS) {
        if(dsme.getDSMESettings().optimizations) {
            if(msg->firstTry) {
                msg->firstTry = false;
                sendMessageDown(msg, false);
                return;
            }
        }

        if(params.gtsTX) { // otherwise failures are common...
            if(params.status == DataStatus::INVALID_GTS) {
                if(queueMessageIfPossible(msg)) {
                    return;
                }

                // GTS slot not yet allocated
                LOG_DEBUG("DROPPED->" << params.msduHandle->getHeader().getDestAddr().getShortAddress() << ": No GTS are currently allocated and upper layer queue full");
            }
            else if(params.status == DataStatus::NO_ACK) {
                // This should not happen, but might be the case for temporary inconsistent slots
                LOG_DEBUG("DROPPED->" << params.msduHandle->getHeader().getDestAddr().getShortAddress() << ": No ACK");
            }
            else if(params.status == DataStatus::TRANSACTION_OVERFLOW){
                // Queue is full
                LOG_DEBUG("DROPPED->" << params.msduHandle->getHeader().getDestAddr().getShortAddress() << ": Queue full");
            }
            else{
                // Should not occur
                DSME_ASSERT(false);
            }
        }
        // TODO specialize!
        // TODO verify that Ack in GTS is always successful for simulation
    }

    if(params.gtsTX) {
        gtsAllocationHelper.indicateOutgoingMessage(params.msduHandle->getHeader().getDestAddr().getShortAddress());
    }


    callback_confirm(params.msduHandle, params.status);
}

void DSMEAdaptionLayer::handleScanComplete(PANDescriptor* panDescriptor) {

    if (panDescriptor != nullptr) {
        LOG_INFO("PAN found on channel " << (uint16_t) panDescriptor->channelNumber << ", coordinator is "
                << panDescriptor->coordAddress.getShortAddress() << ".");
        this->associationInProgress = true;
        this->scanInProgress = false;
        this->associationHelper.associate(panDescriptor->coordPANId, panDescriptor->coordAddrMode, panDescriptor->coordAddress,
                panDescriptor->channelNumber);
    } else {
        LOG_INFO("Channel scan did not yield any PANs." << " Trying again!");
        this->scanHelper.startScan();
    }
    return;
}

void DSMEAdaptionLayer::handleAssociationComplete(AssociationStatus::Association_Status status) {
    this->associationInProgress = false;
    if (status == AssociationStatus::SUCCESS) {
        LOG_INFO("Association completed successfully.");
    } else {
        LOG_INFO("Association failed.");
    }
    return;
}

} /* dsme */
