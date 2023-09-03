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

#ifndef GTSSCHEDULINGIMPL_H_
#define GTSSCHEDULINGIMPL_H_

#include "GTSScheduling.h"
#include "../DSMEAdaptionLayer.h"
#include "../../mac_services/pib/MAC_PIB.h"

namespace dsme {
template <typename SchedulingData, typename RxData>
class GTSSchedulingImpl : public GTSScheduling {
public:
    typedef typename RBTree<SchedulingData, uint16_t>::iterator iterator;

    GTSSchedulingImpl(DSMEAdaptionLayer& dsmeAdaptionLayer) : GTSScheduling(dsmeAdaptionLayer) {
    }

    virtual ~GTSSchedulingImpl() = default;

    virtual void reset() {
        while(this->txLinks.size() > 0) {
            auto it = this->txLinks.begin();
            this->txLinks.remove(it);
        }
        while(this->rxLinks.size() > 0) {
            auto it = this->rxLinks.begin();
            this->rxLinks.remove(it);
        }
    }

    virtual uint8_t registerIncomingMessage(uint16_t address) {
        queueLevel++;
        iterator it = this->txLinks.find(address);
        if(it == this->txLinks.end()) {
            SchedulingData data;
            data.address = address;
            data.messagesInLastMultisuperframe++;
            this->txLinks.insert(data, address);
        } else {
            it->messagesInLastMultisuperframe++;
        }
        return queueLevel;
    }

    virtual void registerOutgoingMessage(uint16_t address, bool success, int32_t serviceTime, uint8_t queueAtCreation) {
        queueLevel--;
        iterator it = this->txLinks.find(address);
        if(it != this->txLinks.end()) {
            it->messagesOutLastMultisuperframe++;
        }

        return;
    }

    virtual void registerReceivedMessage(uint16_t address) {
        auto it = this->rxLinks.find(address);
        if(it == this->rxLinks.end()) {
            RxData data;
            data.address = address;
            data.messagesRxLastMultisuperframe++;
            this->rxLinks.insert(data, address);
        } else {
            it->messagesRxLastMultisuperframe++;
        }
    }

    virtual int16_t getSlotTarget(uint16_t address) {
        iterator it = this->txLinks.find(address);

        return it->slotTarget;
    }

    static uint16_t abs(int16_t v) {
        if(v > 0) {
            return v;
        } else {
            return -v;
        }
    }

    uint16_t getPriorityLink() {
        uint16_t address = IEEE802154MacAddress::NO_SHORT_ADDRESS;
        int16_t difference = 0;
        for(const SchedulingData& d : this->txLinks) {
            uint16_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(d.address, Direction::TX);

            if(abs(difference) < abs(d.slotTarget - slots)) {
                difference = d.slotTarget - slots;
                address = d.address;
            }
        }
        return address;
    }

    virtual GTSSchedulingDecision getNextSchedulingAction(uint16_t address) {
        uint16_t numAllocatedSlots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(address, Direction::TX);

        int16_t target = getSlotTarget(address);

        if(target > numAllocatedSlots) {
            uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
            uint8_t randomSuperframeID = this->dsmeAdaptionLayer.getRandom() % numSuperFramesPerMultiSuperframe;

            uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(randomSuperframeID);
            uint8_t randomSlotID = this->dsmeAdaptionLayer.getRandom() % numGTSlots;

            return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1, randomSuperframeID, randomSlotID};
        } else if(target < numAllocatedSlots && numAllocatedSlots > 1) {
            uint8_t dealloc_num = numAllocatedSlots - target;
            /* TODO: slot and superframe ID are currently ignored for DEALLOCATION */
            return GTSSchedulingDecision{address, ManagementType::DEALLOCATION, Direction::TX, dealloc_num, 0, 0};
        } else {
            return NO_SCHEDULING_ACTION;
        }
    }

    virtual GTSSchedulingDecision getNextSchedulingAction() {
        uint16_t address = getPriorityLink();
        if(address != IEEE802154MacAddress::NO_SHORT_ADDRESS) {
            return getNextSchedulingAction(address);
        } else {
            return NO_SCHEDULING_ACTION;
        }
    }

protected:
    RBTree<SchedulingData, uint16_t> txLinks;
    RBTree<RxData, uint16_t> rxLinks;
    uint8_t queueLevel = 0;
};

} /* namespace dsme */

#endif /* GTSSCHEDULINGIMPL_H_ */
