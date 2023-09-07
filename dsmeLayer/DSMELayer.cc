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

#include "./DSMELayer.h"

#include "opendsme/dsme_platform.h"
#include "opendsme/dsme_settings.h"
#include "../helper/ChannelHoppingLFSR.h"
#include "../helper/DSMEAtomic.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/pib/MAC_PIB.h"
#include "../mac_services/pib/PIBHelper.h"
#include "../mac_services/pib/dsme_mac_constants.h"

#ifdef EVAL_DSME_MSF_DMM_INSTRUMENTATION
#include "eval_utils.h"
extern bool _msf_instr_enabled;
extern bool _first_msf_notified;
extern bool _last_msf_notified;
extern uint32_t _msf_to_measure_count;
#endif

namespace dsme {

DSMELayer::DSMELayer()
    : phy_pib(nullptr),
      mac_pib(nullptr),
      mcps_sap(nullptr),
      mlme_sap(nullptr),

      platform(nullptr),
      eventDispatcher(*this),

      ackLayer(*this),
      capLayer(*this),
      associationManager(*this),
      beaconManager(*this),
      gtsManager(*this),
      messageDispatcher(*this),

      currentSlot(0),
      currentSuperframe(0),
      currentMultiSuperframe(0),
      nextSlot(0),
      nextSuperframe(0),
      nextMultiSuperframe(0),
      trackingBeacons(false),
      nextSlotTime(0),
      resetPending(false) {
}

void DSMELayer::initialize(IDSMEPlatform* platform) {
    this->platform = platform;

    /* only beacons are expected in the beginning.
     * Once the MAC is synchronized the receive delegate is switched to the CAP
     * layer, which also handles (forwards) MAC commands for association */
    platform->setReceiveDelegate(DELEGATE(&BeaconManager::onReceive, beaconManager));

    this->currentSlot = 0;
    this->currentSuperframe = 0;
    this->currentMultiSuperframe = 0;

    this->eventDispatcher.initialize();
    this->gtsManager.initialize();
    this->messageDispatcher.initialize();
    this->beaconManager.initialize();

    this->mac_pib->macDsn = platform->getRandom();

    if(this->mac_pib->macChannelDiversityMode == Channel_Diversity_Mode::CHANNEL_HOPPING) {
        /* compute default hopping sequence */
        this->mac_pib->macHoppingSequenceLength = mac_pib->helper.getNumChannels();
        this->mac_pib->macHoppingSequenceList.setLength(this->mac_pib->macHoppingSequenceLength);
        for(int i = 0; i < this->mac_pib->macHoppingSequenceLength; i++) {
            this->mac_pib->macHoppingSequenceList[i] = this->mac_pib->helper.getChannels()[i];
        }
        ChannelHoppingLFSR lfsr;
        for(int i = 0; i < this->mac_pib->macHoppingSequenceLength; i++) {
            uint8_t ch_cpy = this->mac_pib->macHoppingSequenceList[i];
            uint8_t shuffle = lfsr.next() % this->mac_pib->macHoppingSequenceLength;
            this->mac_pib->macHoppingSequenceList[i] = this->mac_pib->macHoppingSequenceList[shuffle];
            this->mac_pib->macHoppingSequenceList[shuffle] = ch_cpy;
        }
    }
}

void DSMELayer::start() {
    if(getMAC_PIB().macIsPANCoord) {
        getMAC_PIB().macIsCoord = true;
    }

    /* start the timer initially */
    this->nextSlotTime = this->eventDispatcher.setupSlotTimer(this->platform->getSymbolCounter(), 0);
}

void DSMELayer::reset() {
    DSME_ASSERT(!resetPending);
    resetPending = true;
}

void DSMELayer::doReset() {
    DSME_ASSERT(resetPending);
    LOG_ERROR("Performing a complete reset of the DSME MLME.");

    DSME_ATOMIC_BLOCK {
        this->ackLayer.reset();

        /* stop all timers */
        this->eventDispatcher.reset();

        this->beaconManager.reset();
        this->associationManager.reset();
        this->gtsManager.reset();
        this->messageDispatcher.reset();

        this->capLayer.reset();

        this->mac_pib->macDsn = platform->getRandom();

        this->currentSlot = 0;
        this->currentSuperframe = 0;
        this->currentMultiSuperframe = 0;

        this->trackingBeacons = false;
    }

    /* restart slot timer */
    this->nextSlotTime = this->eventDispatcher.setupSlotTimer(this->platform->getSymbolCounter(), 0);

    mlme_sap::RESET_confirm_parameters confirm_params;
    confirm_params.status = ResetStatus::SUCCESS;
    this->getMLME_SAP().getRESET().notify_confirm(confirm_params);

    resetPending = false;
}

void DSMELayer::preSlotEvent(void) {
    if(resetPending) {
        doReset();
        return;
    }

    uint32_t lkb = beaconManager.getLastKnownBeaconIntervalStart();
    /* calculate a symbol counter that is within the next slot (the slot that is about to happen after
     * this pre-slot event).
     * Offset current symbol counter enough to also guarantee it points to the next slot even if
     * the timer indicates a few symbols before the slot.
     * This is needed if the timer triggers a bit too early or indicates a slightly lower time
     * e.g. due to quantization of a 32 kHz timer.
     * This can be done safely because afterwards cnt is divided by SymbolsPerSlot anyway. */
    uint32_t cnt = platform->getSymbolCounter() - lkb + PRE_EVENT_SHIFT + (getMAC_PIB().helper.getSymbolsPerSlot() / 2);

    // calculate slot position
    uint16_t slotsSinceLastKnownBeaconIntervalStart = cnt / getMAC_PIB().helper.getSymbolsPerSlot();
    nextSlot = slotsSinceLastKnownBeaconIntervalStart % aNumSuperframeSlots;
    uint16_t superframe = slotsSinceLastKnownBeaconIntervalStart / aNumSuperframeSlots;
    nextSuperframe = superframe % getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
    superframe /= getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
    nextMultiSuperframe = superframe % getMAC_PIB().helper.getNumberMultiSuperframesPerBeaconInterval();

    if(nextSlot == 0) {
        beaconManager.preSuperframeEvent(nextSuperframe, nextMultiSuperframe, nextSlotTime);
    }

    messageDispatcher.handlePreSlotEvent(nextSlot, nextSuperframe, nextMultiSuperframe);
}

uint8_t DSMELayer::getSkippableSlots(uint16_t currentSlot, uint16_t currentSuperframe) {
#if DSME_MINIMIZE_CFP_SLOT_EVENTS == 0
    /* whithout slot minimization, just CAP slots are skipped */
    uint8_t skippedSlots = 0;
    if(currentSlot == 1) { // beginning of CAP
        if(this->mac_pib->macCapReduction && currentSuperframe > 0) {
            // no CAP available
            skippedSlots = 0;
        } else {
            // no (pre) slot events required during CAP
            skippedSlots = 7;
        }
    }
    return skippedSlots;
#else
    /* whith slot minimization, identify which is the next slot that requires action
     * and skip all slots till then */
    DSMEAllocationCounterTable& act = getMAC_PIB().macDSMEACT;

    /* if the current slot event is an allocated GTS the next slot event
     * has to be active at least to turn off the radio again. */
    if (currentSlot > getMAC_PIB().helper.getFinalCAPSlot(currentSuperframe)) {
      unsigned gtsid = currentSlot - getMAC_PIB().helper.getFinalCAPSlot(currentSuperframe) - 1;
      if (act.isAllocated(currentSuperframe, gtsid)) {
        return 0;
      }
    }
    uint8_t offset = 1;
    unsigned SFperMSF = getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
    /* iterate over all upcoming slots till we find one that requires action */
    while (true) {
      uint8_t ns = (currentSlot + offset) % 16;
      uint16_t nSF = currentSuperframe;
      if (ns < currentSlot) {
        /* wrap araound to next SF */
        nSF = (nSF + 1) % SFperMSF;
      }
      if (ns == 0) { /* beacon slot */
        break;
      } else if ((ns == 1)) { /* (potentially) start of CAP */
        if (!getMAC_PIB().macCapReduction || (nSF == 0)) {
          /* '-> next superframe contains a CAP phase -> skip there */
          break;
        }
      } else if (ns == (getMAC_PIB().helper.getFinalCAPSlot(currentSuperframe) + 1)) {
          /* always wakeup for the first CFP slot to handle radio powerdown
           * and startOfCFPEvent properly */
          break;
      }
      /* if next slot is is GTS slot -> check if it is allocated.. */
      if (ns > getMAC_PIB().helper.getFinalCAPSlot(currentSuperframe)) {
        unsigned gtsid = ns - getMAC_PIB().helper.getFinalCAPSlot(currentSuperframe) - 1;
        if (act.isAllocated(currentSuperframe, gtsid)) {
          /* ... and schedule delayed wakeup for next allocated GTS */
          break;
        }
      }
      offset++;
    }
    return offset - 1;
#endif
}

void DSMELayer::slotEvent(int32_t lateness) {
    if(resetPending) {
        doReset();
        return;
    }

    currentSlot = nextSlot;
    currentSuperframe = nextSuperframe;
    currentMultiSuperframe = nextMultiSuperframe;

    if(getMAC_PIB().macIsPANCoord) {
        LOG_DEBUG(DECOUT << currentSlot << " " << currentSuperframe << " " << currentMultiSuperframe);
    }

    // As long as the lateness is not bigger than the slot, this won't break
    // anything - the slot event will still check if the transmission fits within the slot
    // and abort if necessary.
    if(lateness > 100) { // TODO reduce
        LOG_ERROR("lateness " << lateness);
        printf("LATE %ld\n", lateness);
        //DSME_ASSERT(false);
    }

    uint32_t currentSlotTime;

    // TODO set timer to next relevant slot only!
    // TODO in that case currentSlot might be used even if no slotEvent was called before -> calculate then
    if(this->trackingBeacons) {
        auto now = platform->getSymbolCounter();
        currentSlotTime = now - (now - beaconManager.getLastKnownBeaconIntervalStart()) % getMAC_PIB().helper.getSymbolsPerSlot();
    } else {
        currentSlotTime = this->nextSlotTime;
    }

    uint8_t skippedSlots = getSkippableSlots(currentSlot, currentSuperframe);
    this->nextSlotTime = eventDispatcher.setupSlotTimer(currentSlotTime, skippedSlots);

    /* handle slot */
    if(currentSlot == 0) {
#ifdef EVAL_DSME_MSF_DMM_INSTRUMENTATION
        if (_msf_instr_enabled && (currentSuperframe == 0)) {
            /* DMM Indication is only done on the RFD, and only if synced */
            if (!getMAC_PIB().macIsPANCoord && getBeaconManager().isSynced()) {
                if (!_first_msf_notified) {
                    NOTIFY_START_TO_DMM;
                    _first_msf_notified = true;
                } else if (!_last_msf_notified) {
                    _msf_to_measure_count--;
                    if (_msf_to_measure_count == 0) {
                      NOTIFY_STOP_TO_DMM;
                      _last_msf_notified = true;
                    }
                }
            }
        }
#endif /* EVAL_DSME_MSF_DMM_INSTRUMENTATION */

        beaconManager.superframeEvent(lateness, currentSlotTime);
    }

    messageDispatcher.handleSlotEvent(currentSlot, currentSuperframe, lateness);

    if(currentSlot == getMAC_PIB().helper.getFinalCAPSlot(currentSuperframe) + 1) {
        platform->scheduleStartOfCFP();
    }
}

void DSMELayer::handleStartOfCFP() {
#ifdef STATISTICS_MONITOR_LATENESS
    if(latenessStatisticsCount++ % 10 == 0) {
        this->eventDispatcher.printLatenessHistogram();
    }
#endif

    if(this->startOfCFPDelegate) {
        this->startOfCFPDelegate();
    }

    this->capLayer.handleStartOfCFP();
    this->gtsManager.handleStartOfCFP(this->currentSuperframe);
    this->associationManager.handleStartOfCFP(this->currentSuperframe);
    this->beaconManager.handleStartOfCFP(this->currentSuperframe, this->currentMultiSuperframe);
}

uint32_t DSMELayer::getSymbolsSinceCapFrameStart(uint32_t time) {
    uint32_t symbolsSinceLastBeaconInterval = time - this->beaconManager.getLastKnownBeaconIntervalStart();

    if(this->mac_pib->macCapReduction) {
        uint32_t symbolsPerMultiSuperframe = aNumSuperframeSlots * (uint32_t)aBaseSlotDuration * (1 << (uint32_t) this->mac_pib->macMultiSuperframeOrder);
        uint32_t symbolsSinceLastMultiSuperframeStart = symbolsSinceLastBeaconInterval % symbolsPerMultiSuperframe;
        return symbolsSinceLastMultiSuperframeStart;
    } else {
        uint32_t symbolsPerSuperframe = aNumSuperframeSlots * (uint32_t)aBaseSlotDuration * (1 << (uint32_t) this->mac_pib->macSuperframeOrder);
        uint32_t symbolsSinceLastSuperframeStart = symbolsSinceLastBeaconInterval % symbolsPerSuperframe;
        return symbolsSinceLastSuperframeStart;
    }
}

bool DSMELayer::isWithinCAP(uint32_t time, uint16_t duration) {
    uint32_t symbolsPerSlot = getMAC_PIB().helper.getSymbolsPerSlot();

    uint32_t symbolsSinceCapFrameStart = getSymbolsSinceCapFrameStart(time);

    uint32_t capStart = symbolsPerSlot; // after beacon slot
    uint32_t capEnd = symbolsPerSlot * (getMAC_PIB().helper.getFinalCAPSlot(0) + 1) - PRE_EVENT_SHIFT;  //TODO: IS THIS CORRECT????

    return (symbolsSinceCapFrameStart >= capStart)              // after beacon slot
           && (symbolsSinceCapFrameStart + duration <= capEnd); // before pre-event of first GTS
}

bool DSMELayer::isWithinTimeSlot(uint32_t now, uint16_t duration) {
    uint32_t symbolsPerSlot = getMAC_PIB().helper.getSymbolsPerSlot();
    uint32_t symbolsSinceLastBeaconInterval = now - this->beaconManager.getLastKnownBeaconIntervalStart();

    uint32_t timeSlotStart = (symbolsSinceLastBeaconInterval / symbolsPerSlot) * symbolsPerSlot + this->beaconManager.getLastKnownBeaconIntervalStart();
    uint32_t timeSlotEnd = timeSlotStart + symbolsPerSlot - PRE_EVENT_SHIFT;

    //TODO: Check if this can  be enabled again.
    //      removed the assert for now because it also triggers if now falls into the
    //      PRE_EVENT_SHIFT part, even though, the below 'fits' check would still correctly determine
    //      that the packet wont fit into the slot anymore.
    //
    //DSME_ASSERT(now >= timeSlotStart && now <= timeSlotEnd);
    LOG_DEBUG("Checking isWithingTimeSlot: slot start time (" << timeSlotStart << ") <= current time (" << now << ") <= duration ("
        << now+duration << ") <= slot end time (" << timeSlotEnd << ")");

    bool fits = (now + duration) <= timeSlotEnd;
    if (!fits) {
      uint32_t slotDuration = timeSlotEnd - timeSlotStart;
      if (duration > slotDuration) {
        /* If you get this message you need to check all timing configurations because
         * something is completely wrong :)
         * This effectively means, that no matter at what point in time this check
         * is performed, the packet will not (never ever) fit into the effective slot time. */
        LOG_DEBUG("Slot only has " <<  slotDuration " symbols but TX would take " << duration);
        DSME_ASSERT(false);
      }
    }
    return fits;
}

void DSMELayer::startTrackingBeacons() {
    this->trackingBeacons = true;
    return;
}

void DSMELayer::stopTrackingBeacons() {
    this->trackingBeacons = false;
    return;
}

bool DSMELayer::isTrackingBeacons() const {
    return this->trackingBeacons;
}

} /* namespace dsme */
