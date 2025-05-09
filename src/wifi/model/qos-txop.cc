/*
 * Copyright (c) 2006, 2009 INRIA
 * Copyright (c) 2009 MIRKO BANCHI
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Authors: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *          Mirko Banchi <mk.banchi@gmail.com>
 *          Stefano Avallone <stavalli@unina.it>
 */

#include "qos-txop.h"

#include "channel-access-manager.h"
#include "ctrl-headers.h"
#include "mac-tx-middle.h"
#include "mgt-action-headers.h"
#include "mpdu-aggregator.h"
#include "msdu-aggregator.h"
#include "wifi-mac-queue-scheduler.h"
#include "ns3/wifi-net-device.h"
#include "wifi-mac-queue.h"
#include "wifi-mac-trailer.h"
#include "wifi-mpdu.h"
#include "wifi-phy.h"
#include "wifi-psdu.h"
#include "wifi-tx-parameters.h"

#include "ns3/ht-configuration.h"
#include "ns3/ht-frame-exchange-manager.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include <iostream>
#include <ostream>

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT WIFI_TXOP_NS_LOG_APPEND_CONTEXT

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("QosTxop");

NS_OBJECT_ENSURE_REGISTERED(QosTxop);

TypeId
QosTxop::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::QosTxop")
            .SetParent<ns3::Txop>()
            .SetGroupName("Wifi")
            .AddConstructor<QosTxop>()
            .AddAttribute("UseExplicitBarAfterMissedBlockAck",
                          "Specify whether explicit BlockAckRequest should be sent upon missed "
                          "BlockAck Response.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&QosTxop::m_useExplicitBarAfterMissedBlockAck),
                          MakeBooleanChecker())
            .AddAttribute("AddBaResponseTimeout",
                          "The timeout to wait for ADDBA response after the Ack to "
                          "ADDBA request is received.",
                          TimeValue(MilliSeconds(5)),
                          MakeTimeAccessor(&QosTxop::SetAddBaResponseTimeout,
                                           &QosTxop::GetAddBaResponseTimeout),
                          MakeTimeChecker())
            .AddAttribute(
                "FailedAddBaTimeout",
                "The timeout after a failed BA agreement. During this "
                "timeout, the originator resumes sending packets using normal "
                "MPDU. After that, BA agreement is reset and the originator "
                "will retry BA negotiation.",
                TimeValue(MilliSeconds(200)),
                MakeTimeAccessor(&QosTxop::SetFailedAddBaTimeout, &QosTxop::GetFailedAddBaTimeout),
                MakeTimeChecker())
            .AddAttribute("BlockAckManager",
                          "The BlockAckManager object.",
                          PointerValue(),
                          MakePointerAccessor(&QosTxop::m_baManager),
                          MakePointerChecker<BlockAckManager>())
            .AddAttribute("NMaxInflights",
                          "The maximum number of links (in the range 1-15) on which an MPDU can be "
                          "simultaneously in-flight.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&QosTxop::m_nMaxInflights),
                          MakeUintegerChecker<uint8_t>(1, 15))
            .AddTraceSource("TxopTrace",
                            "Trace source for TXOP start and duration times",
                            MakeTraceSourceAccessor(&QosTxop::m_txopTrace),
                            "ns3::QosTxop::TxopTracedCallback")        
            .AddTraceSource("GetNextParams","The statistics of the BAW",
                        MakeTraceSourceAccessor(&QosTxop::TracedParamsAndStats),
                        "ns3::QosTxop::ParamTracedCallback")
            .AddTraceSource("GetTxopTimeStats","The statistics of txop time",
                MakeTraceSourceAccessor(&QosTxop::TracedTxopTime),
                "ns3::QosTxop::TxopTimeStatsTracedCallback");
                
            
    return tid;
}

QosTxop::QosTxop()
{
    NS_LOG_FUNCTION(this);
    m_baManager = CreateObject<BlockAckManager>();
    m_link_up = 0b11;
    m_alg_txop_limits = {0, 0};
    m_alg_rts_cts_thresholds = {-1, -1};
}

void
QosTxop::CreateQueue(AcIndex aci)
{   
    NS_LOG_FUNCTION(this << aci);
    Txop::CreateQueue(aci);
    m_ac = aci;
    m_baManager->SetQueue(m_queue);
    m_baManager->SetBlockDestinationCallback(
        Callback<void, Mac48Address, uint8_t>([this](Mac48Address recipient, uint8_t tid) {
            m_mac->GetMacQueueScheduler()->BlockQueues(WifiQueueBlockedReason::WAITING_ADDBA_RESP,
                                                       m_ac,
                                                       {WIFI_QOSDATA_QUEUE},
                                                       recipient,
                                                       m_mac->GetLocalAddress(recipient),
                                                       {tid});
        }));
    m_baManager->SetUnblockDestinationCallback(
        Callback<void, Mac48Address, uint8_t>([this](Mac48Address recipient, uint8_t tid) {
            // save the status of AC queues before unblocking the transmissions to the recipient
            std::map<uint8_t, bool> hasFramesToTransmit;
            for (const auto& [id, link] : GetLinks())
            {
                hasFramesToTransmit[id] = HasFramesToTransmit(id);
            }

            m_mac->GetMacQueueScheduler()->UnblockQueues(WifiQueueBlockedReason::WAITING_ADDBA_RESP,
                                                         m_ac,
                                                         {WIFI_QOSDATA_QUEUE},
                                                         recipient,
                                                         m_mac->GetLocalAddress(recipient),
                                                         {tid});

            // start access (if needed) on all the links
            for (const auto& [id, link] : GetLinks())
            {
                StartAccessAfterEvent(id, hasFramesToTransmit.at(id), CHECK_MEDIUM_BUSY);
            }
        }));
    m_queue->TraceConnectWithoutContext(
        "Expired",
        MakeCallback(&BlockAckManager::NotifyDiscardedMpdu, m_baManager));
}

QosTxop::~QosTxop()
{
    NS_LOG_FUNCTION(this);
}

void
QosTxop::DoDispose()
{
    NS_LOG_FUNCTION(this);
    if (m_baManager)
    {
        m_baManager->Dispose();
    }
    m_baManager = nullptr;
    Txop::DoDispose();
}

void
QosTxop::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    Txop::DoInitialize();
    m_baManager->SetMode(m_mode);
    if(m_mode & 0x03){
        for(uint8_t i = 0; i < m_mac->GetNLinks(); i++)
        {
            m_mac->GetFrameExchangeManager(i)->TraceConnectWithoutContext("AckedMpdu",
                MakeCallback(&MsduGrouper::NotifyAcked, GetMsduGrouper()));
            m_mac->GetDevice()->GetPhy(i)->TraceConnectWithoutContext("PpduTxDuration",MakeCallback(&MsduGrouper::NotifyPpduTxDuration, GetMsduGrouper()));
            m_mac->GetDevice()->GetPhy(i)->TraceConnectWithoutContext("MonitorSnifferTx",MakeCallback(&MsduGrouper::NotifyPhyTxEvent, GetMsduGrouper()));
        }
        m_baManager->TraceConnectWithoutContext(
            "AckedMpdu",
            MakeCallback(&MsduGrouper::NotifyAcked, GetMsduGrouper()));
        m_baManager->TraceConnectWithoutContext(
            "BawDiscardMpdu",
            MakeCallback(&MsduGrouper::NotifyDiscardedMpdu, GetMsduGrouper()));

        if(m_ac == AC_BE) {
            Simulator::Schedule(Seconds(3), &QosTxop::ScheduleUpdateEdcaParameters, this, GetMsduGrouper()->GetPeriod()); 
        }
    }
}

std::unique_ptr<Txop::LinkEntity>
QosTxop::CreateLinkEntity() const
{
    return std::make_unique<QosLinkEntity>();
}

QosTxop::QosLinkEntity&
QosTxop::GetLink(uint8_t linkId) const
{
    return static_cast<QosLinkEntity&>(Txop::GetLink(linkId));
}

uint8_t
QosTxop::GetQosQueueSize(uint8_t tid, Mac48Address receiver) const
{
    WifiContainerQueueId queueId{WIFI_QOSDATA_QUEUE, WIFI_UNICAST, receiver, tid};
    uint32_t bufferSize = m_queue->GetNBytes(queueId);
    // A queue size value of 254 is used for all sizes greater than 64 768 octets.
    uint8_t queueSize = static_cast<uint8_t>(std::ceil(std::min(bufferSize, 64769U) / 256.0));
    NS_LOG_DEBUG("Buffer size=" << bufferSize << " Queue Size=" << +queueSize);
    return queueSize;
}

void
QosTxop::SetDroppedMpduCallback(DroppedMpdu callback)
{
    NS_LOG_FUNCTION(this << &callback);
    Txop::SetDroppedMpduCallback(callback);
    m_baManager->SetDroppedOldMpduCallback(callback.Bind(WIFI_MAC_DROP_QOS_OLD_PACKET));
}

void
QosTxop::SetMuCwMin(uint16_t cwMin, uint8_t linkId)
{
    NS_LOG_FUNCTION(this << cwMin << +linkId);
    GetLink(linkId).muCwMin = cwMin;
}

void
QosTxop::SetMuCwMax(uint16_t cwMax, uint8_t linkId)
{
    NS_LOG_FUNCTION(this << cwMax << +linkId);
    GetLink(linkId).muCwMax = cwMax;
}

void
QosTxop::SetMuAifsn(uint8_t aifsn, uint8_t linkId)
{
    NS_LOG_FUNCTION(this << +aifsn << +linkId);
    GetLink(linkId).muAifsn = aifsn;
}

void
QosTxop::SetMuEdcaTimer(Time timer, uint8_t linkId)
{
    NS_LOG_FUNCTION(this << timer << +linkId);
    GetLink(linkId).muEdcaTimer = timer;
}

void
QosTxop::StartMuEdcaTimerNow(uint8_t linkId)
{
    NS_LOG_FUNCTION(this << +linkId);
    auto& link = GetLink(linkId);
    link.muEdcaTimerStartTime = Simulator::Now();
    if (EdcaDisabled(linkId))
    {
        NS_LOG_DEBUG("Disable EDCA for " << link.muEdcaTimer.As(Time::MS));
        m_mac->GetChannelAccessManager(linkId)->DisableEdcaFor(this, link.muEdcaTimer);
    }
}

bool
QosTxop::MuEdcaTimerRunning(uint8_t linkId) const
{
    auto& link = GetLink(linkId);
    return (link.muEdcaTimerStartTime.IsStrictlyPositive() &&
            link.muEdcaTimer.IsStrictlyPositive() &&
            link.muEdcaTimerStartTime + link.muEdcaTimer > Simulator::Now());
}

bool
QosTxop::EdcaDisabled(uint8_t linkId) const
{
    return (MuEdcaTimerRunning(linkId) && GetLink(linkId).muAifsn == 0);
}

uint32_t
QosTxop::GetMinCw(uint8_t linkId) const
{
    if (!MuEdcaTimerRunning(linkId))
    {
        return GetLink(linkId).cwMin;
    }
    NS_ASSERT(!EdcaDisabled(linkId));
    return GetLink(linkId).muCwMin;
}

uint32_t
QosTxop::GetMaxCw(uint8_t linkId) const
{
    if (!MuEdcaTimerRunning(linkId))
    {
        return GetLink(linkId).cwMax;
    }
    NS_ASSERT(!EdcaDisabled(linkId));
    return GetLink(linkId).muCwMax;
}

uint8_t
QosTxop::GetAifsn(uint8_t linkId) const
{
    if (!MuEdcaTimerRunning(linkId))
    {
        return GetLink(linkId).aifsn;
    }
    return GetLink(linkId).muAifsn;
}

Ptr<BlockAckManager>
QosTxop::GetBaManager()
{
    return m_baManager;
}

uint16_t
QosTxop::GetBaBufferSize(Mac48Address address, uint8_t tid) const
{
    return m_baManager->GetRecipientBufferSize(address, tid);
}

uint16_t
QosTxop::GetBaStartingSequence(Mac48Address address, uint8_t tid) const
{
    return m_baManager->GetOriginatorStartingSequence(address, tid);
}

uint16_t
QosTxop::GetBaStartingSequence(Mac48Address address, uint8_t tid, uint8_t linkId) const
{
    if (m_mode & 0x03) return m_baManager->GetOriginatorRptr(address, tid, linkId);
    return m_baManager->GetOriginatorStartingSequence(address, tid);
}


std::pair<CtrlBAckRequestHeader, WifiMacHeader>
QosTxop::PrepareBlockAckRequest(Mac48Address recipient, uint8_t tid) const
{
    NS_LOG_FUNCTION(this << recipient << +tid);
    NS_ASSERT(QosUtilsMapTidToAc(tid) == m_ac);

    auto recipientMld = m_mac->GetMldAddress(recipient);

    CtrlBAckRequestHeader reqHdr =
        m_baManager->GetBlockAckReqHeader(recipientMld.value_or(recipient), tid);

    WifiMacHeader hdr;
    hdr.SetType(WIFI_MAC_CTL_BACKREQ);
    hdr.SetAddr1(recipient);
    hdr.SetAddr2(m_mac->GetLocalAddress(recipient));
    hdr.SetDsNotTo();
    hdr.SetDsNotFrom();
    hdr.SetNoRetry();
    hdr.SetNoMoreFragments();

    return {reqHdr, hdr};
}

bool
QosTxop::UseExplicitBarAfterMissedBlockAck() const
{
    return m_useExplicitBarAfterMissedBlockAck;
}

bool
QosTxop::HasFramesToTransmit(uint8_t linkId)
{
    // remove MSDUs with expired lifetime starting from the head of the queue
    m_queue->WipeAllExpiredMpdus();
    auto hasFramesToTransmit = static_cast<bool>(m_queue->PeekFirstAvailable(linkId));

    // Print the number of packets that are actually in the queue (which might not be
    // eligible for transmission for some reason, e.g., TID not mapped to the link, etc.)
    NS_LOG_DEBUG(m_ac << " on link " << +linkId << (hasFramesToTransmit ? " has" : " has not")
                      << " frames to transmit with " << m_queue->GetNPackets()
                      << " packets in the queue");
    return hasFramesToTransmit;
}

uint16_t
QosTxop::GetNextSequenceNumberFor(const WifiMacHeader* hdr)
{
    return m_txMiddle->GetNextSequenceNumberFor(hdr);
}

uint16_t
QosTxop::PeekNextSequenceNumberFor(const WifiMacHeader* hdr)
{
    return m_txMiddle->PeekNextSequenceNumberFor(hdr);
}

bool
QosTxop::IsQosOldPacket(Ptr<const WifiMpdu> mpdu)
{
    NS_LOG_FUNCTION(this << *mpdu);

    if (!mpdu->GetHeader().IsQosData())
    {
        return false;
    }

    Mac48Address recipient = mpdu->GetHeader().GetAddr1();
    uint8_t tid = mpdu->GetHeader().GetQosTid();

    if (!m_mac->GetBaAgreementEstablishedAsOriginator(recipient, tid))
    {
        return false;
    }

    return QosUtilsIsOldPacket(GetBaStartingSequence(recipient, tid),
                               mpdu->GetHeader().GetSequenceNumber());
}

Ptr<WifiMpdu>
QosTxop::PeekNextMpdu(uint8_t linkId, uint8_t tid, Mac48Address recipient, Ptr<const WifiMpdu> mpdu)
{
    NS_LOG_FUNCTION(this << +linkId << +tid << recipient << mpdu);

    // lambda to peek the next frame
    auto peek = [this, &linkId, &tid, &recipient, &mpdu]() -> Ptr<WifiMpdu> {
        if (tid == 8 && recipient.IsBroadcast()) // undefined TID and recipient
        {
            return m_queue->PeekFirstAvailable(linkId, mpdu);
        }
        WifiContainerQueueId queueId(WIFI_QOSDATA_QUEUE, WIFI_UNICAST, recipient, tid);
        if (auto mask = m_mac->GetMacQueueScheduler()->GetQueueLinkMask(m_ac, queueId, linkId);
            mask && mask->none())
        {
            return m_queue->PeekByQueueId(queueId, mpdu);
        }
        return nullptr;
    };

    auto item = peek();
    // remove old packets (must be retransmissions or in flight, otherwise they did
    // not get a sequence number assigned)
    while (item && !item->IsFragment())
    {
        if (item->GetHeader().IsCtl())
        {
            NS_LOG_DEBUG("Skipping control frame: " << *item);
            mpdu = item;
            item = peek();
            continue;
        }

        if (item->HasSeqNoAssigned() && IsQosOldPacket(item))
        {
            NS_LOG_DEBUG("Removing an old packet from EDCA queue: " << *item);
            if (!m_droppedMpduCallback.IsNull())
            {
                m_droppedMpduCallback(WIFI_MAC_DROP_QOS_OLD_PACKET, item);
            }
            mpdu = item;
            item = peek();
            m_queue->Remove(mpdu);
            continue;
        }

        if ( m_mode & 0x02 )
        {   
            // std::cout << "mode 2" << IsLinkAllocated(linkId, item->GetAllocatedLink()) << std::endl;
            if(!IsLinkAllocated(linkId, item->GetAllocatedLink()) && item->GetPacket()->GetAdjustment() != item->GetPacketSize())
            {
                // peek the next sequence number and check if it is within the transmit window
                WifiMacHeader& hdr = item->GetHeader();
                // in case of QoS data frame
                uint16_t sequence = item->HasSeqNoAssigned() ? hdr.GetSequenceNumber()
                                                            : m_txMiddle->PeekNextSequenceNumberFor(&hdr);
                if (hdr.IsQosData())
                {
                    Mac48Address recipient = hdr.GetAddr1();
                    uint8_t tid = hdr.GetQosTid();

                    if (m_mac->GetBaAgreementEstablishedAsOriginator(recipient, tid) &&
                        !IsInWindow(sequence,
                                    GetBaStartingSequence(recipient, tid),
                                    GetBaBufferSize(recipient, tid)))
                    {
                        NS_LOG_DEBUG("Packet beyond the end of the current transmit window");
                        // std::cout<<"Search mpdu for link "<<+linkId<<std::endl;
                        // std::cout<<"Packet beyond the end of the current transmit window"<<std::endl;
                        // std::cout<<"BaStartingSequence "<<GetBaStartingSequence(recipient, tid)<<std::endl;
                        // std::cout<<"BaBufferSize "<<GetBaBufferSize(recipient, tid)<<std::endl;
                        return nullptr; //如果此MPDU已经超出窗口,后面将要搜索的MPDU肯定也以已经超出
                    }
                }

                bool firstAssignSeqNo = (!item->IsFragment() && !item->HasSeqNoAssigned());
                AssignSequenceNumber(item); //真正分配序列号  
                GetMsduGrouper()->NotifyPacketEnqueue(item, firstAssignSeqNo);
                NS_LOG_DEBUG("Skipping frames that are not assigned to this link " << item->GetHeader().GetSequenceNumber());
                mpdu = item;
                item = peek();
                continue;
            }
            if (auto linkIds = item->GetInFlightLinkIds(); !linkIds.empty()) // MPDU is in-flight
            {
                // This MPDU has already been sent on other links, but can still be sent on this link.
                // mode2 中在其他链路上发送过的肯定是未分配到当前链路上的
                // 冗余个数不如设置为另一条链路的平均聚合个数*失败率
                if (!linkIds.contains(linkId))
                {
                    NS_LOG_DEBUG(*item <<" This MPDU has already been sent on other links, but can still be sent on link "<<+linkId);
                    break;
                }
                // if no BA agreement, we cannot have multiple MPDUs in-flight
                if (item->GetHeader().IsQosData() &&
                    !m_mac->GetBaAgreementEstablishedAsOriginator(item->GetHeader().GetAddr1(),
                                                                item->GetHeader().GetQosTid()))
                {
                    NS_LOG_DEBUG("No BA agreement and an MPDU is already in-flight");
                    return nullptr;
                }

                NS_LOG_DEBUG("Skipping in flight MPDU: " << *item);
                mpdu = item;
                item = peek();
                continue;
            }
        }
        else if ( m_mode & 0x01 )
        {
            // if (m_link_up & (1 << linkId) != 1) return nullptr;
            if (auto linkIds = item->GetInFlightLinkIds(); !linkIds.empty()) // MPDU is in-flight
            {
                // 此MPDU已经在其他链路上发送,如果此链路冗余，可以在此链路上发送
                if (!linkIds.contains(linkId))
                {
                    GetMsduGrouper()->m_inflighted[*linkIds.begin()] ++; 
                    if(GetMsduGrouper() && GetMsduGrouper()->GetRedundancyMode(linkId) && GetMsduGrouper()->AvailableRedundancy(linkId))
                    {
                        NS_LOG_DEBUG("link" << +linkId << " 冗余，"
                                            << item->GetHeader().GetSequenceNumber() << "再次发送");
                        // std::cout << "The MPDU is inflighted on Link " << (uint32_t) (* linkIds.begin()) << ", but can be sent on Link " << (uint32_t)linkId << ", the SN is " << item->GetHeader().GetSequenceNumber()  << " cnt: " << GetMsduGrouper().m_RedundantPacketCnt[linkId] << std::endl;
                        break;
                    }
                }
                // if no BA agreement, we cannot have multiple MPDUs in-flight
                if (item->GetHeader().IsQosData() &&
                    !m_mac->GetBaAgreementEstablishedAsOriginator(item->GetHeader().GetAddr1(),
                                                                item->GetHeader().GetQosTid()))
                {
                    NS_LOG_DEBUG("No BA agreement and an MPDU is already in-flight");
                    return nullptr;
                }

                NS_LOG_DEBUG("Skipping in flight MPDU: " << *item);
                mpdu = item;
                item = peek();
                continue;
            }
        }
        else  // 默认模式
        {
            if (auto linkIds = item->GetInFlightLinkIds(); !linkIds.empty()) // MPDU is in-flight
            {
                // m_nMaxInflights=2
                // 此MPDU已经在其他链路上发送,可以在此链路上发送
                if (!linkIds.contains(linkId) && (linkIds.size() < m_nMaxInflights))
                {
                    break;
                }
                // if no BA agreement, we cannot have multiple MPDUs in-flight
                if (item->GetHeader().IsQosData() &&
                    !m_mac->GetBaAgreementEstablishedAsOriginator(item->GetHeader().GetAddr1(),
                                                                item->GetHeader().GetQosTid()))
                {
                    NS_LOG_DEBUG("No BA agreement and an MPDU is already in-flight");
                    return nullptr;
                }

                NS_LOG_DEBUG("Skipping in flight MPDU: " << *item);
                mpdu = item;
                item = peek();
                continue;
            }
        }

        if (item->GetHeader().HasData() &&
            !m_mac->CanForwardPacketsTo(item->GetHeader().GetAddr1()))
        {
            NS_LOG_DEBUG("Skipping frame that cannot be forwarded: " << *item);
            mpdu = item;
            item = peek();
            continue;
        }
        break;
    }

    if (!item)
    {
        return nullptr;
    }

    WifiMacHeader& hdr = item->GetHeader();

    // peek the next sequence number and check if it is within the transmit window
    // in case of QoS data frame
    uint16_t sequence = item->HasSeqNoAssigned() ? hdr.GetSequenceNumber()
                                                 : m_txMiddle->PeekNextSequenceNumberFor(&hdr);
    if (hdr.IsQosData())
    {
        Mac48Address recipient = hdr.GetAddr1();
        uint8_t tid = hdr.GetQosTid();
        if (m_mac->GetBaAgreementEstablishedAsOriginator(recipient, tid) &&
            !IsInWindow(sequence,
                        GetBaStartingSequence(recipient, tid, linkId),
                        GetBaBufferSize(recipient, tid)))
        {
            NS_LOG_DEBUG("Packet beyond the end of the current transmit window");
            return nullptr;
        }
    }

    // Assign a sequence number if this is not a fragment nor it already has one assigned
    if (!item->IsFragment() && !item->HasSeqNoAssigned())
    {        
        hdr.SetSequenceNumber(sequence);
    }
    NS_LOG_DEBUG("Packet peeked from EDCA queue: " << *item);
    return item;
}

bool 
QosTxop::IsLinkAllocated(uint8_t linkId, uint8_t allocatedlink)
{
    if (linkId == 0) {
        return (allocatedlink == 1 || allocatedlink == 3);
    } else if (linkId == 1) {
        return (allocatedlink == 2 || allocatedlink == 3);
    }
    return false;
}

Ptr<WifiMpdu>
QosTxop::GetNextMpdu(uint8_t linkId,
                     Ptr<WifiMpdu> peekedItem,
                     WifiTxParameters& txParams,
                     Time availableTime,
                     bool initialFrame)
{
    NS_ASSERT(peekedItem);
    NS_LOG_FUNCTION(this << +linkId << *peekedItem << &txParams << availableTime << initialFrame);

    Mac48Address recipient = peekedItem->GetHeader().GetAddr1();

    // The TXOP limit can be exceeded by the TXOP holder if it does not transmit more
    // than one Data or Management frame in the TXOP and the frame is not in an A-MPDU
    // consisting of more than one MPDU (Sec. 10.22.2.8 of 802.11-2016)
    Time actualAvailableTime =
        (initialFrame && txParams.GetSize(recipient) == 0 ? Time::Min() : availableTime);

    auto qosFem = StaticCast<QosFrameExchangeManager>(m_mac->GetFrameExchangeManager(linkId));
    if (!qosFem->TryAddMpdu(peekedItem, txParams, actualAvailableTime))
    {
        return nullptr;
    }

    NS_ASSERT(peekedItem->IsQueued());
    Ptr<WifiMpdu> mpdu;

    // If it is a non-broadcast QoS Data frame and it is not a retransmission nor a fragment,
    // attempt A-MSDU aggregation
    if (peekedItem->GetHeader().IsQosData())
    {
        uint8_t tid = peekedItem->GetHeader().GetQosTid();

        // we should not be asked to dequeue an MPDU that is beyond the transmit window.
        // Note that PeekNextMpdu() temporarily assigns the next available sequence number
        // to the peeked frame
        NS_ASSERT(!m_mac->GetBaAgreementEstablishedAsOriginator(recipient, tid) ||
                  IsInWindow(
                      peekedItem->GetHeader().GetSequenceNumber(),
                      GetBaStartingSequence(peekedItem->GetOriginal()->GetHeader().GetAddr1(), tid),
                      GetBaBufferSize(peekedItem->GetOriginal()->GetHeader().GetAddr1(), tid)));

        // try A-MSDU aggregation if the MPDU does not contain an A-MSDU and does not already
        // have a sequence number assigned (may be a retransmission)
        if (m_mac->GetHtConfiguration() && !recipient.IsBroadcast() &&
            !peekedItem->GetHeader().IsQosAmsdu() && !peekedItem->HasSeqNoAssigned() &&
            !peekedItem->IsFragment())
        {
            auto htFem = StaticCast<HtFrameExchangeManager>(qosFem);
            mpdu = htFem->GetMsduAggregator()->GetNextAmsdu(peekedItem, txParams, availableTime);
        }

        if (mpdu)
        {
            NS_LOG_DEBUG("Prepared an MPDU containing an A-MSDU");
        }
        // else aggregation was not attempted or failed
    }

    if (!mpdu)
    {
        mpdu = peekedItem; 
    }

    if(mpdu->GetHeader().IsQosData() && m_mode & 0x03)
    {
        // 打断当前分组，增加组号
        Ptr<QosTxop> edca = m_mac->GetQosTxop(mpdu->GetHeader().GetQosTid());
        edca->GetMsduGrouper()->AddCurrentGroup(mpdu->GetOriginal()->GetGroupNumber());
    }

    // Assign a sequence number if this is not a fragment nor a retransmission
    bool firstAssignSeqNo = (!mpdu->IsFragment() && !mpdu->HasSeqNoAssigned());
    AssignSequenceNumber(mpdu); // 模式二下，曾经跳过发送的MPDU已经分配序列号，不会再次分配

    NS_LOG_DEBUG("Got MPDU from EDCA queue: " << *mpdu);
    if (m_mode & 0x03)
        GetMsduGrouper()->NotifyPacketEnqueue(mpdu, firstAssignSeqNo);
    return mpdu;
}

void
QosTxop::AssignSequenceNumber(Ptr<WifiMpdu> mpdu) const
{
    NS_LOG_FUNCTION(this << *mpdu);
    if (!mpdu->IsFragment() && !mpdu->HasSeqNoAssigned())
    {
        // in case of 11be MLDs, sequence numbers refer to the MLD address
        auto origMpdu = m_queue->GetOriginal(mpdu);
        uint16_t sequence = m_txMiddle->GetNextSequenceNumberFor(&origMpdu->GetHeader());
        mpdu->AssignSeqNo(sequence);
    }
}

void
QosTxop::NotifyChannelAccessed(uint8_t linkId, Time txopDuration)
{
    NS_LOG_FUNCTION(this << +linkId << txopDuration);

    NS_ASSERT(txopDuration != Time::Min());
    GetLink(linkId).startTxop = Simulator::Now();
    GetLink(linkId).txopDuration = txopDuration;
    Txop::NotifyChannelAccessed(linkId);
    if (m_mode & 0x03)
        GetMsduGrouper()->m_txopTimeBegin[linkId] = Simulator::Now().GetMicroSeconds();
}

std::optional<Time>
QosTxop::GetTxopStartTime(uint8_t linkId) const
{
    auto& link = GetLink(linkId);
    NS_LOG_FUNCTION(this << link.startTxop.has_value());
    return link.startTxop;
}

void
QosTxop::NotifyChannelReleased(uint8_t linkId)
{
    NS_LOG_FUNCTION(this << +linkId);
    auto& link = GetLink(linkId);

    if (link.startTxop)
    {
        NS_LOG_DEBUG("Terminating TXOP. Duration = " << Simulator::Now() - *link.startTxop);
        m_txopTrace(*link.startTxop, Simulator::Now() - *link.startTxop, linkId);
    }

    // generate a new backoff value if either the TXOP duration is not null (i.e., some frames
    // were transmitted) or no frame was transmitted but the queue actually contains frame to
    // transmit and the user indicated that a backoff value should be generated in this situation.
    // This behavior reflects the following specs text (Sec. 35.3.16.4 of 802.11be D4.0):
    // An AP or non-AP STA affiliated with an MLD that has gained the right to initiate the
    // transmission of a frame as described in 10.23.2.4 (Obtaining an EDCA TXOP) for an AC but
    // does not transmit any frame corresponding to that AC for the reasons stated above may:
    // - invoke a backoff for the EDCAF associated with that AC as allowed per h) of 10.23.2.2
    //   (EDCA backoff procedure).
    auto hasTransmitted = link.startTxop.has_value() && Simulator::Now() > *link.startTxop;

    m_queue->WipeAllExpiredMpdus();
    if ((hasTransmitted) ||
        (!m_queue->IsEmpty() && m_mac->GetChannelAccessManager(linkId)->GetGenerateBackoffOnNoTx()))
    {
        GenerateBackoff(linkId);
        if (!m_queue->IsEmpty())
        {
            Simulator::ScheduleNow(&QosTxop::RequestAccess, this, linkId);
        }
    }
    link.startTxop.reset();
    GetLink(linkId).access = NOT_REQUESTED;

    if (m_mode & 0x03) {
        GetMsduGrouper()->SetTxopTimeEnd(Simulator::Now().GetMicroSeconds(), linkId);
        if(GetMsduGrouper()->IsParamUpdateEnabled()) {
            SetTxopLimit(MicroSeconds(m_alg_txop_limits[linkId]) * 32, linkId);
        }
    }

}

Time
QosTxop::GetRemainingTxop(uint8_t linkId) const
{
    auto& link = GetLink(linkId);
    NS_ASSERT(link.startTxop.has_value());

    Time remainingTxop = link.txopDuration;
    remainingTxop -= (Simulator::Now() - *link.startTxop);
    if (remainingTxop.IsStrictlyNegative())
    {
        remainingTxop = Seconds(0);
    }
    NS_LOG_FUNCTION(this << remainingTxop);
    return remainingTxop;
}

void
QosTxop::GotAddBaResponse(const MgtAddBaResponseHeader& respHdr, Mac48Address recipient)
{
    NS_LOG_FUNCTION(this << respHdr << recipient);
    uint8_t tid = respHdr.GetTid();

    if (respHdr.GetStatusCode().IsSuccess())
    {
        NS_LOG_DEBUG("block ack agreement established with " << recipient << " tid " << +tid);
        // A (destination, TID) pair is "blocked" (i.e., no more packets are sent) when an
        // Add BA Request is sent to the destination. However, when the Add BA Request timer
        // expires, the (destination, TID) pair is "unblocked" and packets to the destination are
        // sent again (under normal ack policy). Thus, there may be a packet with a sequence number
        // already assigned waiting to be retransmitted (or being transmitted on another link)
        // when the Add BA Response is received. In this case, the starting sequence number shall
        // be set equal to the sequence number of such packet.
        uint16_t startingSeq = m_txMiddle->GetNextSeqNumberByTidAndAddress(tid, recipient);
        auto peekedItem = m_queue->PeekByTidAndAddress(tid, recipient);
        if (peekedItem && peekedItem->HasSeqNoAssigned())
        {
            startingSeq = peekedItem->GetHeader().GetSequenceNumber();
        }
        m_baManager->UpdateOriginatorAgreement(respHdr, recipient, startingSeq);
    }
    else
    {
        NS_LOG_DEBUG("discard ADDBA response" << recipient);
        m_baManager->NotifyOriginatorAgreementRejected(recipient, tid);
    }
}

void
QosTxop::GotDelBaFrame(const MgtDelBaHeader* delBaHdr, Mac48Address recipient)
{
    NS_LOG_FUNCTION(this << delBaHdr << recipient);
    NS_LOG_DEBUG("received DELBA frame from=" << recipient);
    m_baManager->DestroyOriginatorAgreement(recipient, delBaHdr->GetTid());
}

void
QosTxop::NotifyOriginatorAgreementNoReply(const Mac48Address& recipient, uint8_t tid)
{
    NS_LOG_FUNCTION(this << recipient << tid);
    m_baManager->NotifyOriginatorAgreementNoReply(recipient, tid);
}

void
QosTxop::CompleteMpduTx(Ptr<WifiMpdu> mpdu)
{
    NS_ASSERT(mpdu->GetHeader().IsQosData());
    // If there is an established BA agreement, store the packet in the queue of outstanding packets
    if (m_mac->GetBaAgreementEstablishedAsOriginator(mpdu->GetHeader().GetAddr1(),
                                                     mpdu->GetHeader().GetQosTid()))
    {
        NS_ASSERT(mpdu->IsQueued());
        NS_ASSERT(m_queue->GetAc() == mpdu->GetQueueAc());
        m_baManager->StorePacket(m_queue->GetOriginal(mpdu));
    }
}

void
QosTxop::SetBlockAckThreshold(uint8_t threshold)
{
    NS_LOG_FUNCTION(this << +threshold);
    m_blockAckThreshold = threshold;
    m_baManager->SetBlockAckThreshold(threshold);
}

void
QosTxop::SetBlockAckInactivityTimeout(uint16_t timeout)
{
    NS_LOG_FUNCTION(this << timeout);
    m_blockAckInactivityTimeout = timeout;
}

uint8_t
QosTxop::GetBlockAckThreshold() const
{
    NS_LOG_FUNCTION(this);
    return m_blockAckThreshold;
}

uint16_t
QosTxop::GetBlockAckInactivityTimeout() const
{
    return m_blockAckInactivityTimeout;
}

void
QosTxop::AddBaResponseTimeout(Mac48Address recipient, uint8_t tid)
{
    NS_LOG_FUNCTION(this << recipient << +tid);
    // If agreement is still pending, ADDBA response is not received
    if (auto agreement = m_baManager->GetAgreementAsOriginator(recipient, tid);
        agreement && agreement->get().IsPending())
    {
        NotifyOriginatorAgreementNoReply(recipient, tid);
        Simulator::Schedule(m_failedAddBaTimeout, &QosTxop::ResetBa, this, recipient, tid);
    }
}

void
QosTxop::ResetBa(Mac48Address recipient, uint8_t tid)
{
    NS_LOG_FUNCTION(this << recipient << +tid);
    // This function is scheduled when waiting for an ADDBA response. However,
    // before this function is called, a DELBA request may arrive, which causes
    // the agreement to be deleted. Hence, check if an agreement exists before
    // notifying that the agreement has to be reset.
    if (auto agreement = m_baManager->GetAgreementAsOriginator(recipient, tid);
        agreement && !agreement->get().IsEstablished())
    {
        m_baManager->NotifyOriginatorAgreementReset(recipient, tid);
    }
}

void
QosTxop::SetAddBaResponseTimeout(Time addBaResponseTimeout)
{
    NS_LOG_FUNCTION(this << addBaResponseTimeout);
    m_addBaResponseTimeout = addBaResponseTimeout;
}

Time
QosTxop::GetAddBaResponseTimeout() const
{
    return m_addBaResponseTimeout;
}

void
QosTxop::SetFailedAddBaTimeout(Time failedAddBaTimeout)
{
    NS_LOG_FUNCTION(this << failedAddBaTimeout);
    m_failedAddBaTimeout = failedAddBaTimeout;
}

Time
QosTxop::GetFailedAddBaTimeout() const
{
    return m_failedAddBaTimeout;
}

bool
QosTxop::IsQosTxop() const
{
    return true;
}

AcIndex
QosTxop::GetAccessCategory() const
{
    return m_ac;
}

void 
QosTxop::ScheduleUpdateEdcaParameters(Time period)
{
    if (!(m_mode & 0x03) || m_mac->GetNLinks() < 2)
        return;
    if (m_ac != AC_BE)
        return;
    std::cout<<"************Time: "<<Simulator::Now().GetSeconds()<<"s************"<<std::endl;
    std::cout << "s, MAC ADDR: " << m_mac->GetAddress() << ": ("<< m_mode << "," << m_ac << ")" << std::endl;

    std::cout <<"-----set-----"<<std::endl;
    std::cout << "MAC ADDR: " << m_mac->GetAddress() << ": (" << m_mode << "," << m_ac << ")"<< std::endl;
    std::cout << "CWmins " << GetMinCws()[0] << " " << GetMinCws()[1] << std::endl;
    std::cout << "CWmaxs " << GetMaxCws()[0] << " " << GetMaxCws()[1] << std::endl; // 15, 1023
    std::cout << "Aifsns " << (uint32_t)GetAifsns()[0] << " " << (uint32_t)GetAifsns()[1] << std::endl; // 3,3
    std::cout << "TxopLimits " << GetTxopLimits()[0] << " " << GetTxopLimits()[1] << std::endl; // 0ns
    std::cout << "BE_MaxAmpduSize " << m_mac->GetMaxAmpduSize(m_ac) << std::endl; 
    std::cout << "link1Pct " << GetMsduGrouper()->GetLink1Pct() << std::endl; 
    std::cout << "MaxGroupSize " << GetMsduGrouper()->GetMaxGroupSize() << std::endl; 
    double Thp1 = GetMsduGrouper()->GetQueueStats().GetThroughput(0x00);
    double Thp2 = GetMsduGrouper()->GetQueueStats().GetThroughput(0x01);
    double s1 = GetMsduGrouper()->GetQueueStats().GetMpduSuccessRate(0x00);
    double s2 = GetMsduGrouper()->GetQueueStats().GetMpduSuccessRate(0x01);
    double chanrate1 = GetMsduGrouper()->GetQueueStats().GetChannelEfficiency(0x00);
    double chanrate2 = GetMsduGrouper()->GetQueueStats().GetChannelEfficiency(0x01);
    double avgdatarate1 = GetMsduGrouper()->GetQueueStats().GetAverageDataRate(0x00);
    double avgdatarate2 = GetMsduGrouper()->GetQueueStats().GetAverageDataRate(0x01);
    std::vector<double> blocktimerate = GetMsduGrouper()->GetQueueStats().GetBlockTimeRateAndClear();

    auto blockCnt = GetMsduGrouper()->GetQueueStats().m_blockCnt;
    auto blockCnt_prev = GetMsduGrouper()->GetQueueStats().m_blockCnt_prev;
    std::vector<uint32_t> res_blockCnt = {blockCnt[0] - blockCnt_prev[0],
                                          blockCnt[1] - blockCnt_prev[1]};
    GetMsduGrouper()->GetQueueStats().m_blockCnt_prev = blockCnt;
    auto blockCnt_tr = GetMsduGrouper()->GetQueueStats().m_blockCnt_tr;
    auto blockCnt_tr_prev = GetMsduGrouper()->GetQueueStats().m_blockCnt_tr_prev;
    GetMsduGrouper()->GetQueueStats().m_blockCnt_tr_prev = blockCnt_tr;
    std::vector<uint32_t> res_blockCnt_tr = {blockCnt_tr[0] - blockCnt_tr_prev[0],
                                             blockCnt_tr[1] - blockCnt_tr_prev[1]};

    auto blockrate = GetMsduGrouper()->GetMeanBlockRate();

    auto bawqueue = GetMsduGrouper()->GetQueueStats().GetBawQueue();
    std::cout << "************Time: " << Simulator::Now().GetSeconds() << "s************"
              << std::endl;
    std::cout << Simulator::Now().GetSeconds() << "s, MAC ADDR: " << m_mac->GetAddress() << ": ("
              << m_mode << "," << m_ac << ")" << std::endl;
    std::cout << "Window Blocked Count: " << blockCnt[0] << " , " << blockCnt[1] << std::endl;
    std::cout << "Thp1: " << Thp1 << "Mbps, Thp2: " << Thp2 << "Mbps" << std::endl;
    std::cout << "s1: " << s1 << ", s2: " << s2 << std::endl;
    std::cout << "chanrate1: " << chanrate1 << ", chanrate2: " << chanrate2 << "" << std::endl;
    std::cout << "avgdatarate1: " << avgdatarate1 << "Mbps, avgdatarate2: " << avgdatarate2
              << "Mbps" << std::endl;
    std::cout << "blocktimerate: " << blocktimerate[0] << ", " << blocktimerate[1] << std::endl;
    std::cout << "blockrate: " << blockrate[0] << ", " << blockrate[1] << std::endl;
    // for(auto it = bawqueue.begin(); it != bawqueue.end(); it++)
    // {
    //     std::cout << "BawQueue: " << it->first.first << " " << (int)it->first.second << std::endl;
    //     for(auto it2 = it->second.begin(); it2 != it->second.end(); it2++)
    //     {
    //         std::cout << "BawQueueItem: " << it2->sepNum << " retry: " << (int)it2->retryState << " assign: " << (int)it2->assignState << " acked: " << it2->acked << " discarded: " << it2->discarded << std::endl;
    //     }
    // }
    std::cout<<"\n";

    // Update Params
    std::vector<uint32_t> maxAmpduLength = GetMsduGrouper()->GetMaxAmpduLength();
    std::vector<uint32_t> meanAmpduLength = GetMsduGrouper()->GetMeanAmpduLength();
    std::cout << "blockCnt: " << res_blockCnt[0] << " , " << res_blockCnt[1] << std::endl;
    std::cout << "blockCnt_tr: " << res_blockCnt_tr[0] << " , " << res_blockCnt_tr[1] << std::endl;
    std::cout << "maxAmpduLength: " << maxAmpduLength[0] << ", " << maxAmpduLength[1] << std::endl;
    std::cout << "meanAmpduLength: " << meanAmpduLength[0] << ", " << meanAmpduLength[1] << std::endl;
    if (GetMsduGrouper()->IsGridSearchEnabled()) {
        std::unordered_map<std::string, std::vector<uint32_t>> params = GetMsduGrouper()->GetCurrentEdcaParameters();
        std::unordered_map<std::string, std::vector<uint32_t>> next_params = GetMsduGrouper()->GetNextEdcaParameters();
        auto & txopTimeList = GetMsduGrouper()->m_txopTimeList;
        auto txoptime1 = std::accumulate(txopTimeList[0].begin(), txopTimeList[0].end(), 0) / (txopTimeList[0].size() == 0 ? 1 : txopTimeList[0].size());
        auto txoptime2 = std::accumulate(txopTimeList[1].begin(), txopTimeList[1].end(), 0) / (txopTimeList[1].size() == 0 ? 1 : txopTimeList[1].size());
        uint32_t txopnum1 = txopTimeList[0].size();
        uint32_t txopnum2 = txopTimeList[1].size();
        if (params.empty()) {
            params["No"] = {0};
            params["CWmins"] = GetMinCws();
            params["CWmaxs"] = GetMaxCws();
            auto txoplimits = GetTxopLimits();
            std::transform(txoplimits.begin(), txoplimits.end(), std::back_inserter(params["TxopLimits"]), [](const ns3::Time& t) { return t.GetMicroSeconds();});
            auto aifsns = GetAifsns();
            params["Aifsns"] = {(uint32_t)aifsns[0], (uint32_t)aifsns[1]};
            params["AmpduSizes"] = {m_mac->GetMaxAmpduSize(m_ac)};
            params["RTS_CTS"] = {0, 0};
            auto tmp1 = m_mac->GetWifiRemoteStationManager(0)->GetMaxSsrcAndMaxSlrcAndRtsCtsThreshold();
            auto tmp2 = m_mac->GetWifiRemoteStationManager(1)->GetMaxSsrcAndMaxSlrcAndRtsCtsThreshold();
            params["MaxSsrcs"] = {tmp1[0], tmp2[0]};
            params["MaxSlrcs"] = {tmp1[1], tmp2[1]};
            params["RedundancyThresholds"] = {50, 50};
            params["RedundancyFixedNumbers"] = {0, 0};
        }
        if (!TracedParamsAndStats.IsEmpty())
            TracedParamsAndStats(params,
                                 GetMsduGrouper()->GetLink1Pct(),
                                 {Thp1, Thp2},
                                 {s1, s2},
                                 {chanrate1, chanrate2},
                                 {avgdatarate1, avgdatarate2},
                                 blocktimerate,
                                 blockrate,
                                 res_blockCnt,
                                 res_blockCnt_tr,
                                 {txoptime1, txoptime2},
                                 {txopnum1, txopnum2},
                                 maxAmpduLength,
                                 meanAmpduLength);
        if (!TracedTxopTime.IsEmpty())
            TracedTxopTime(GetMsduGrouper()->m_txopList, GetMsduGrouper()->m_txopNumList);

        if (!next_params.size()) {
            Simulator::Schedule(period, &QosTxop::ScheduleUpdateEdcaParameters, this, period);
            return;
        }
        if (GetMsduGrouper()->IsParamUpdateEnabled()) {
            // CWmin, CWmax
            SetMinCws(next_params["CWmins"]);
            SetMaxCws(next_params["CWmaxs"]);

            // TxopLimits
            m_alg_txop_limits = next_params["TxopLimits"];

            // Aifsns
            SetAifsns(std::vector<uint8_t>(next_params["Aifsns"].begin(), next_params["Aifsns"].end()));

            // RTS/CTS 开启关闭
            for (uint8_t i = 0; i < 2; ++ i) {
                if (next_params["RTS_CTS"][i])
                    m_mac->GetWifiRemoteStationManager(i)->SetRtsCtsThreshold(0);
                else m_mac->GetWifiRemoteStationManager(i)->SetRtsCtsThreshold(std::numeric_limits<uint32_t>::max());
            }

            // 聚合参数

            // m_mac->SetAttribute("BE_MaxAmpduSize", UintegerValue(65536));

            // 重传次数
            for (uint8_t i = 0; i < 2; ++ i) {
                m_mac->GetWifiRemoteStationManager(i)->SetMaxSlrc(params["MaxSlrcs"][i]); // 7
                m_mac->GetWifiRemoteStationManager(i)->SetMaxSsrc(params["MaxSsrcs"][i]); // 4
            }
        
            // 冗余参数
            GetMsduGrouper()->UpdateRedundancyThreshold(next_params["RedundancyThresholds"]);

            GetMsduGrouper()->UpdateRedundancyFixedNumber(next_params["RedundancyFixedNumbers"][0]);

        }
        GetMsduGrouper()->ClearStats();
    } else {
        std::unordered_map<std::string, std::vector<uint32_t>> params = GetMsduGrouper()->GetNewEdcaParameters();


        if (!params.size()) {
            Simulator::Schedule(period, &QosTxop::ScheduleUpdateEdcaParameters, this, period);
            return;
        }

        // CWmin, CWmax
        // SetMinCws(params["CWmins"]);
        // SetMaxCws(params["CWmaxs"]);

        // TxopLimits
        // std::vector<Time> new_txopLimits;
        // for (auto t : params["TxopLimits"]) {
        //     new_txopLimits.push_back(MicroSeconds(t)); // us
        // }
        // SetTxopLimits(new_txopLimits); // 32 us的整数倍

        // Aifsns
        // SetAifsns(std::vector<uint8_t>(params["Aifsns"].begin(), params["Aifsns"].end()));

        // RTS/CTS 开启关闭
        // for (uint8_t i = 0; i < 2; ++ i) {
        //     if (params["RTS_CTS"][i])
        //         m_mac->GetWifiRemoteStationManager(i)->SetRtsCtsThreshold(0);
        //     else m_mac->GetWifiRemoteStationManager(i)->SetRtsCtsThreshold(std::numeric_limits<uint32_t>::max());
        // }

        // 聚合参数

        // m_mac->SetAttribute("BE_MaxAmpduSize", UintegerValue(params["AmpduSizes"][0]));

        // 重传次数
        // for (uint8_t i = 0; i < 2; ++ i) {
        //     m_mac->GetWifiRemoteStationManager(i)->SetMaxSlrc(params["MaxSlrcs"][i]); // 7
        //     m_mac->GetWifiRemoteStationManager(i)->SetMaxSsrc(params["MaxSsrcs"][i]); // 5
        // }

        // 冗余参数
        // GetMsduGrouper().UpdateRedundancyThreshold(params["RedundancyThreshold"]);
    }

    Simulator::Schedule(period, &QosTxop::ScheduleUpdateEdcaParameters, this, period);
}

bool 
QosTxop::IsLinkUp(uint8_t linkId)
{
    if(!(m_mode & 0x03)) return true;
    return m_link_up & (1 << linkId);
}

void 
QosTxop::UpdateLinkStates() {
    std::pair<uint8_t, Time> params = GetMsduGrouper()->GetNewLinkStates();
    if (m_link_up != params.first)
        Simulator::Schedule(params.second, &QosTxop::SetLinkUp, this, params.first);
}

void 
QosTxop::SetLinkUp(uint8_t newLinkUp) {
    m_link_up = newLinkUp;
}

} // namespace ns3
