#include "msdu-grouper.h"

#include "ns3/ampdu-subframe-header.h"
#include "ns3/frame-exchange-manager.h"
#include "ns3/msdu-aggregator.h"
#include "ns3/wifi-tx-vector.h"

#include <iostream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MsduGrouper");
NS_OBJECT_ENSURE_REGISTERED(MsduGrouper);

QueueStats::QueueStats(Time period)
{
    cycle_time = period;
    m_initialized = false;
    m_blockCnt = {0, 0};
    m_blockCnt_prev = {0, 0};
    m_blockCnt_tr = {0, 0};
    m_blockCnt_tr_prev = {0, 0};
    blockwindows = {Seconds(0), Seconds(0)};
    blockwindows_Total = {Seconds(0), Seconds(0)};
}

QueueStats::QueueStats()
{
    cycle_time = Seconds(0.1);
    m_initialized = false;
    blockwindows = {Seconds(0), Seconds(0)};
    blockwindows_Total = {Seconds(0), Seconds(0)};
}

QueueStats::~QueueStats()
{
    m_bawqueue.clear();
}

bool
QueueStats::Initialize()
{
    // TODO: connect to the simulator
    m_initialized = true;
    return m_initialized;
}

double
QueueStats::GetThroughput()
{
    double throughput = 0;
    for (const auto& it : m_mpduinfos)
    {
        if (it.m_rxstate &&
            (!cycle_time.IsStrictlyPositive() || Simulator::Now() - it.m_txTime < cycle_time))
        {
            throughput += it.m_size;
        }
    }
    if (!cycle_time.IsStrictlyPositive())
        return throughput / Simulator::Now().GetSeconds();
    return throughput / cycle_time.GetSeconds();
}

double
QueueStats::GetThroughput(uint8_t linkId)
{
    double throughput = 0;
    for (const auto& it : m_mpduinfos)
    {
        if (it.m_rxstate &&
            (!cycle_time.IsStrictlyPositive() || Simulator::Now() - it.m_txTime < cycle_time) &&
            (it.m_linkIds & (1 << linkId)))
        {
            throughput += it.m_size;
        }
    }
    if (!cycle_time.IsStrictlyPositive())
        return throughput / Simulator::Now().GetSeconds();
    return throughput * 8 / cycle_time.GetMicroSeconds();
}

double
QueueStats::GetChannelEfficiency()
{
    Time totalduration = Seconds(0);
    if (cycle_time.IsStrictlyPositive())
    {
        for (auto it = m_ppduinfos.rbegin(); it != m_ppduinfos.rend(); it++)
        {
            if (Simulator::Now() - it->m_mpduinfos[0].m_txTime < cycle_time)
            {
                totalduration += it->txDuration;
            }
            else
            {
                break;
            }
        }
        return totalduration.GetSeconds() / cycle_time.GetSeconds();
    }
    else
    {
        for (auto it = m_ppduinfos.rbegin(); it != m_ppduinfos.rend(); it++)
        {
            totalduration += it->txDuration;
        }
        return totalduration.GetSeconds() / Simulator::Now().GetSeconds();
    }
    return 0;
}

double
QueueStats::GetChannelEfficiency(uint8_t linkId)
{
    Time totalduration;
    if (cycle_time.IsStrictlyPositive())
    {
        for (auto it = m_ppduinfos.rbegin(); it != m_ppduinfos.rend(); it++)
        {
            if (Simulator::Now() - it->m_mpduinfos[0].m_txTime < cycle_time &&
                (it->m_mpduinfos[0].m_linkIds & (1 << linkId)))
            {
                totalduration += it->txDuration;
            }
        }
        return totalduration.GetSeconds() / cycle_time.GetSeconds();
    }
    else
    {
        for (auto it = m_ppduinfos.rbegin(); it != m_ppduinfos.rend(); it++)
        {
            if ((it->m_mpduinfos[0].m_linkIds & (1 << linkId)))
            {
                totalduration += it->txDuration;
            }
        }
        return totalduration.GetSeconds() / Simulator::Now().GetSeconds();
    }
    return 0;
}

double
QueueStats::GetMpduSuccessRate()
{
    double success = 0;
    double total = 0;
    for (const auto& it : m_mpduinfos)
    {
        if ((!cycle_time.IsStrictlyPositive() || Simulator::Now() - it.m_txTime < cycle_time))
        {
            if (it.m_rxstate)
                success++;
            total++;
        }
    }
    return success / total;
}

double
QueueStats::GetMpduSuccessRate(uint8_t linkId)
{
    double successnum = 0;
    double totalnum = 0;
    for (const auto& it : m_mpduinfos)
    {
        if ((!cycle_time.IsStrictlyPositive() || Simulator::Now() - it.m_txTime < cycle_time) &&
            (it.m_linkIds & (linkId + 1)))
        {
            if (it.m_rxstate)
                successnum++;
            totalnum += it.m_txcount;
        }
    }
    return successnum / totalnum;
}

std::vector<double>
QueueStats::GetRecentAMPDULength(uint8_t linkId)
{
    std::vector<double> lengths;
    if (cycle_time.IsStrictlyPositive())
    {
        for (auto it = m_ppduinfos.rbegin(); it != m_ppduinfos.rend(); it++)
        {
            if ((it->m_mpduinfos[0].m_linkIds & (linkId + 1)) &&
                (Simulator::Now() - it->txTime < cycle_time))
            {
                lengths.push_back(it->m_mpduinfos.size());
            }
        }
    }
    else
    {
        for (auto it = m_ppduinfos.rbegin(); it != m_ppduinfos.rend(); it++)
        {
            if ((it->m_mpduinfos[0].m_linkIds & (linkId + 1)))
            {
                lengths.push_back(it->m_mpduinfos.size());
            }
        }
    }
    return lengths;
}

double
QueueStats::GetAverageDataRate(uint8_t linkId)
{
    double datarate = 0;
    Time totalduration = Seconds(0);
    if (cycle_time.IsStrictlyPositive())
    {
        for (auto it = m_ppduinfos.rbegin(); it != m_ppduinfos.rend(); it++)
        {
            if (it->txDuration.IsStrictlyPositive() &&
                (it->m_mpduinfos[0].m_linkIds & (linkId + 1)) &&
                (Simulator::Now() - it->txTime < cycle_time))
            {
                datarate += it->m_mpduinfos[0].DataRate * it->txDuration.GetSeconds();
                totalduration += it->txDuration;
            }
        }
    }
    else
    {
        for (auto it = m_ppduinfos.rbegin(); it != m_ppduinfos.rend(); it++)
        {
            if (it->txDuration.IsStrictlyPositive() &&
                (it->m_mpduinfos[0].m_linkIds & (linkId + 1)))
            {
                datarate += it->m_mpduinfos[0].DataRate * it->txDuration.GetSeconds();
                totalduration += it->txDuration;
            }
        }
    }
    return datarate / totalduration.GetMicroSeconds();
}

// 入队
bool
QueueStats::Enqueue(Ptr<const WifiMpdu> mpdu)
{
    Mac48Address recipient = mpdu->GetOriginal()->GetHeader().GetAddr1();
    uint8_t tid = mpdu->GetHeader().GetQosTid();
    auto it = m_bawqueue.find({recipient, tid});
    WiFiBawQueueIt mpduit;
    mpduit.seqNo = mpdu->GetHeader().GetSequenceNumber();
    mpduit.assignState = 0;
    mpduit.assignState = mpdu->GetAllocatedLink();
    mpduit.retryState = 0;
    mpduit.acked = false;
    mpduit.discarded = false;
    mpduit.packet = mpdu->GetPacket();
    if (it == m_bawqueue.end())
    {
        m_bawqueue[{recipient, tid}] = std::vector<WiFiBawQueueIt>();
        m_bawqueue[{recipient, tid}].push_back(mpduit);
    }
    else
    {
        it->second.push_back(mpduit);
        if (it->second.size() > 4096)
        {
            it->second.erase(it->second.begin());
        }
    }
    return true;
}

// 出队
bool
QueueStats::Pop(Ptr<const WifiMpdu> mpdu, bool ackordiscard)
{
    if (!mpdu->GetHeader().IsQosData())
        return false;
    Mac48Address recipient = mpdu->GetOriginal()->GetHeader().GetAddr1();
    uint8_t tid = mpdu->GetHeader().GetQosTid();
    if (recipient == Mac48Address("00:00:00:00:00:03") ||
        recipient == Mac48Address("00:00:00:00:00:02"))
    {
        recipient = Mac48Address("00:00:00:00:00:01");
    }
    std::vector<WiFiBawQueueIt>& queueit = m_bawqueue.find({recipient, tid})->second;
    uint16_t seqNo = mpdu->GetHeader().GetSequenceNumber();
    auto mpduit = std::find_if(queueit.begin(), queueit.end(), [&seqNo](const WiFiBawQueueIt& it) {
        return it.seqNo == seqNo;
    });
    mpduit->acked = ackordiscard;
    mpduit->discarded = !ackordiscard;
    NS_ASSERT(mpduit != queueit.end());
    while (queueit.size() > 0)
    {
        if (queueit.front().acked || queueit.front().discarded)
        {
            queueit.erase(queueit.begin());
        }
        else
        {
            break;
        }
    }
    return true;
}

MsduGrouper::MsduGrouper()
    : MsduGrouper(1, 4096, nullptr, nullptr, 0, Seconds(0)) // 委托构造给原构造函数，设置默认参数
{
}

MsduGrouper::MsduGrouper(uint32_t maxGroupSize,
                         uint32_t maxGroupNumber,
                         Ptr<WifiMacQueue> queue,
                         Ptr<WifiMac> mac,
                         uint32_t mode,
                         Time period)
    :
      m_maxGroupSize(maxGroupSize),
      m_queue(queue),
      m_mac(mac),
      m_mode(mode),
      m_period(period),
      m_maxGroupNumber(maxGroupNumber),
      m_currentGroup(0),
      m_currentCount(0),
      m_firstMsdu(nullptr),
      m_link1Pct(0),
      m_enqueueNum(0)
{
    m_allocatedLink1Pr = CreateObject<UniformRandomVariable>();
    m_allocatedLink1Pr->SetAttribute("Min", DoubleValue(0.0));
    m_allocatedLink1Pr->SetAttribute("Max", DoubleValue(1.0));
    m_allocatedLink2Pr = CreateObject<UniformRandomVariable>();
    m_allocatedLink2Pr->SetAttribute("Min", DoubleValue(0.0));
    m_allocatedLink2Pr->SetAttribute("Max", DoubleValue(1.0));

    m_redundancyMode = 0;
    m_redundancyThreshold = {50, 50};
    m_maxAmpduSize = {0, 0};
    m_startTime = Seconds(1);
    m_queueStats = QueueStats(m_period);
    m_maxRedundantPackets = {2, 2};
    m_RedundantPacketCnt = {0, 0};
    m_redundancyFixedNumber = 0;
    m_inflighted = {0, 0};
    m_gs_enable = false;
    m_param_update = false;
}

MsduGrouper::~MsduGrouper()
{
}

TypeId
MsduGrouper::GetTypeId()
{
    static TypeId tid = TypeId("ns3::MsduGrouper")
                            .SetParent<Object>()
                            .SetGroupName("Wifi")
                            .AddConstructor<MsduGrouper>();
    return tid;
}

void
MsduGrouper::AssignAmsduByPr()
{
    double value = m_allocatedLink1Pr->GetValue();
    if (value < m_link1Pct)
    {
        m_firstMsdu->SetAllocatedLink(1);
    }
    else
    {
        m_firstMsdu->SetAllocatedLink(2);
    }
}

void 
MsduGrouper::SetLink1PctByQueuePowAvg(double thp1, double thp2)
{
    std::vector<uint64_t> allocatedLinksNum =
        m_queue->CountAllocatedLinks(*m_queueIds.cbegin()); // MacQueue中分配到各链路的msdu的数量

    // 计算实时比例
    auto assignlink1Num = static_cast<uint64_t>(std::round(
        (thp1 * allocatedLinksNum[1] - thp2 * allocatedLinksNum[0] + thp1 * m_enqueueNum) /
        (thp1 + thp2)));

    double new_pct = 0.0;
    if (m_enqueueNum != 0)
    {
        new_pct = static_cast<double>(assignlink1Num) / static_cast<double>(m_enqueueNum);
        new_pct = std::round(new_pct * 1000.0) / 1000.0;
    }

    // 维护移动窗口
    m_historyPct.push_back(new_pct);
    while (m_historyPct.size() > 5)
    {
        m_historyPct.pop_front();
    }

    double weighted_sum = 0.0;
    double weight_total = 0.0;
    for (size_t i = 0; i < m_historyPct.size(); ++i)
    {
        double weight = i + 1; // 第0个元素（最旧）权重1
        weighted_sum += m_historyPct[i] * weight;
        weight_total += weight;
    }
    m_link1Pct = std::round((weighted_sum / weight_total) * 1000.0) / 1000.0;

    std::cout << "m_enqueueNum: " << m_enqueueNum << std::endl;
    std::cout << "allocated link1: " << allocatedLinksNum[0] << " link2: " << allocatedLinksNum[1]
              << std::endl;
    std::cout << "thp: link1=" << thp1 << " link2=" << thp2 << std::endl;
    std::cout << "new_pct: " << new_pct << " WMA_pct: " << m_link1Pct
              << " (window=" << m_historyPct.size() << ")" << std::endl;
}

void
MsduGrouper::AggregateMsdu(Ptr<WifiMpdu> msdu)
{
    if (m_mode == 0)
        return;
    m_enqueueNum ++;
    m_queueIds.insert(WifiMacQueueContainer::GetQueueId(msdu));
    if (m_queueIds.size() >= 2) {
        NS_LOG_WARN("WifiContainerQueueId: Detected two different queue IDs");
    }

    if (m_currentCount >= m_maxGroupSize)
    {
        // 如果当前组已满，则移动到下一组
        m_currentGroup = (m_currentGroup + 1) % m_maxGroupNumber;
        // 聚合完成入队
        m_firstMsdu = nullptr;
    }

    msdu->SetGroupNumber(m_currentGroup);
    m_currentCount = (m_currentCount + 1) % m_maxGroupSize;
    // 如果当前组只有一个MSDU，则将其设置为第一个MSDU并返回
    if (m_currentCount == 1)
    {
        m_firstMsdu = msdu;
        if (m_mode == 2)
        {
            AssignAmsduByPr();
            return;
        }
    }

    // 如果当前组有多个MSDU，则将当前MSDU与前一个MSDU聚合
    if (m_firstMsdu && m_firstMsdu->GetGroupNumber() == msdu->GetGroupNumber() &&
        m_firstMsdu != msdu)
    {
        // NS_LOG_DEBUG("Aggregate MSDU ( " << msdu <<" groupNumber "<<m_currentGroup<<" ) into
        // AMSDU "<<m_firstMsdu->GetSize());
        m_queue->DequeueIfQueued({m_firstMsdu});
        m_firstMsdu->Aggregate(msdu);
        m_queue->Replace(msdu, m_firstMsdu);
    }
}

void
MsduGrouper::AddCurrentGroup(uint32_t itemgroup)
{
    if (itemgroup == m_currentGroup)
    {
        m_currentGroup = (m_currentGroup + 1) % m_maxGroupNumber;
        m_currentCount = 0;
        m_firstMsdu = nullptr;
        // std::cout<<"current group add to " << m_currentGroup<<std::endl;
        // NS_LOG_DEBUG ("current group add to " << m_currentGroup);
    }
}

// MPDU 入队
void
MsduGrouper::NotifyPacketEnqueue(Ptr<const WifiMpdu> mpdu, bool firstAssignSeqNo)
{
    if (!mpdu->GetHeader().IsQosData())
        return;
    if (firstAssignSeqNo)
    {   
        MPDUInfo mpduinfo;
        mpduinfo.m_Uid = mpdu->GetPacket()->GetUid();
        mpduinfo.m_receiver = mpdu->GetHeader().GetAddr1();
        mpduinfo.m_size = mpdu->GetPacket()->GetSize();
        mpduinfo.m_msduNum = 0;
        mpduinfo.m_mpduSeqNo = mpdu->GetHeader().GetSequenceNumber();
        mpduinfo.m_rxstate = false;
        mpduinfo.m_txcount = 0;
        mpduinfo.m_linkIds = 0;
        mpduinfo.m_ackTime = Seconds(0);
        m_queueStats.m_mpduinfos.push_back(std::move(mpduinfo));
        m_queueStats.Enqueue(mpdu);
    }
}

// PPDU (AMPDU) Tx
void
MsduGrouper::NotifyPpduTxDuration(Ptr<const WifiPpdu> ppdu, Time duration, uint8_t linkId)
{
    if (!ppdu->GetPsdu()->GetHeader(0).IsQosData())
        return;
    PPDUInfo ppduinfo;
    ppduinfo.txDuration = duration;
    ppduinfo.txTime = Simulator::Now();
    Ptr<const WifiPsdu> psdu = ppdu->GetPsdu();
    if (psdu->IsAggregate())
    {
        uint32_t nmpdus = psdu->GetNMpdus();
        for (uint32_t i = 0; i < nmpdus; i++)
        {
            Ptr<Packet> packet = psdu->GetAmpduSubframe(i);
            auto it = std::find_if(
                m_queueStats.m_mpduinfos.rbegin(),
                m_queueStats.m_mpduinfos.rend(),
                [&packet](const MPDUInfo& it) { return it.m_Uid == packet->GetUid(); });
            if (it != m_queueStats.m_mpduinfos.rend())
            {
                it->m_txTime = Simulator::Now();
                ppduinfo.m_mpduinfos.push_back(*it);
            }
            else
            {
                NS_ABORT_MSG("MsduGrouper::NotifyPpduTxDuration: MPDU not found in QueueStats"
                             << Simulator::Now() << " " << packet->GetUid());
            }
        }
    }
    else
    {
        Ptr<const Packet> packet = psdu->GetPacket();
        auto it =
            std::find_if(m_queueStats.m_mpduinfos.rbegin(),
                         m_queueStats.m_mpduinfos.rend(),
                         [&packet](const MPDUInfo& it) { return it.m_Uid == packet->GetUid(); });
        if (it != m_queueStats.m_mpduinfos.rend())
        {
            it->m_txTime = Simulator::Now();
            ppduinfo.m_mpduinfos.push_back(*it);
        }
        else
        {
            NS_ABORT_MSG("MsduGrouper::NotifyPpduTxDuration: MPDU not found in QueueStats"
                         << Simulator::Now() << " " << packet->GetUid());
        }
    }
    m_queueStats.m_ppduinfos.push_back(ppduinfo);
}

// MPDU Discard
void
MsduGrouper::NotifyDiscardedMpdu(Ptr<const WifiMpdu> mpdu)
{
    m_queueStats.Pop(mpdu, false);
}


void
MsduGrouper::NotifyPhyTxEvent(Ptr<const Packet> packet,
                              uint16_t channelFreqMhz,
                              WifiTxVector txVector,
                              MpduInfo aMpdu,
                              uint16_t staId)
{
    uint64_t id = packet->GetUid(); // 获得包的标号
    uint8_t linkId = 0x00;
    if (channelFreqMhz < 3000) // 2.4G链路
    {
        linkId = 0x01;
    }
    else if (channelFreqMhz < 5700) // 5G链路
    {
        linkId = 0x02;
    }
    Ptr<Packet> p = packet->Copy();
    if (txVector.IsAggregation())
    {
        AmpduSubframeHeader subHdr;
        uint32_t extractedLength;
        p->RemoveHeader(subHdr);
        extractedLength = subHdr.GetLength();
        p = p->CreateFragment(0, static_cast<uint32_t>(extractedLength));
    }
    // Mac48Address mldAddress = m_mac->GetMldAddress(staId);
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    if (!hdr.IsQosData() || !hdr.HasData())
        return;
    auto it = std::find_if(m_queueStats.m_mpduinfos.rbegin(),
                           m_queueStats.m_mpduinfos.rend(),
                           [&id](const MPDUInfo& mit) { return mit.m_Uid == id; });
    if (it != m_queueStats.m_mpduinfos.rend())
    {
        if (hdr.IsQosData() && hdr.IsQosAmsdu())
        {
            it->m_msduNum = m_maxGroupSize;
        }
        it->m_linkIds |= linkId;
        it->m_txcount += 1;
        it->DataRate = txVector.GetMode(staId).GetDataRate(txVector.GetChannelWidth(),
                                                           txVector.GetGuardInterval(),
                                                           1) *
                       txVector.GetNss(staId);
    }
    else
    {
        MPDUInfo mpduinfo;
        mpduinfo.m_Uid = id;
        mpduinfo.m_receiver = hdr.GetAddr1();
        mpduinfo.m_size = p->GetSize() - hdr.GetSerializedSize();
        mpduinfo.m_msduNum = 0;
        mpduinfo.m_rxstate = false;
        mpduinfo.m_txcount = 1;
        mpduinfo.m_linkIds = linkId;
        mpduinfo.m_ackTime = Seconds(0);
        if (hdr.IsQosData() && hdr.IsQosAmsdu())
        {
            mpduinfo.m_msduNum = m_maxGroupSize;
        }
        mpduinfo.DataRate = txVector.GetMode(staId).GetDataRate(txVector.GetChannelWidth(),
                                                                txVector.GetGuardInterval(),
                                                                1) * txVector.GetNss(staId);
        m_queueStats.m_mpduinfos.push_back(mpduinfo);
    }
    Mac48Address recipient = hdr.GetAddr1();
    uint8_t tid = hdr.GetQosTid();
    if (recipient == Mac48Address("00:00:00:00:00:03") ||
        recipient == Mac48Address("00:00:00:00:00:02"))
    {
        recipient = Mac48Address("00:00:00:00:00:01");
    }
    auto& queue = m_queueStats.m_bawqueue.find({recipient, tid})->second;
    uint16_t seqNo = hdr.GetSequenceNumber();
    auto queueit = std::find_if(queue.begin(), queue.end(), [&seqNo](const WiFiBawQueueIt& it) {
        return it.seqNo == seqNo;
    });
    if (queueit != queue.end())
    {
        queueit->retryState += 1;
        queueit->assignState = queueit->assignState | linkId;
        queueit->acked = false;
        queueit->discarded = false;
    }
    else
    {
        NS_ABORT_MSG("MsduGrouper::NotifyPhyTxEvent: MPDU not found in BawQueue" << Simulator::Now()
                                                                                 << " " << seqNo);
    }
}

void
MsduGrouper::NotifyAcked(Ptr<const WifiMpdu> mpdu, uint8_t linkId)
{
    uint64_t id = mpdu->GetPacket()->GetUid(); // 获得包号
    auto it = std::find_if(m_queueStats.m_mpduinfos.begin(),
                           m_queueStats.m_mpduinfos.end(),
                           [&id](const MPDUInfo& it) { return it.m_Uid == id; });
    if (it != m_queueStats.m_mpduinfos.end())
    {
        it->m_msduNum = mpdu->GetNMsdus();
        it->m_rxstate = true;
        it->m_ackTime = Simulator::Now();
    }

    m_queueStats.Pop(mpdu, true);
}

bool
MsduGrouper::GetRedundancyMode(uint8_t linkId)
{
    return m_redundancyMode & (1 << linkId);
}

void
MsduGrouper::SetRedundancyMode(uint8_t linkId, uint32_t re_num)
{
    m_redundancyMode = m_redundancyMode | (1 << linkId);
    m_maxRedundantPackets[linkId] = re_num;
}

void
MsduGrouper::ResetRedundancyMode(uint8_t linkId)
{
    // std::cout << "Redundancy mode closed on Link " << uint32_t(linkId) << std::endl;
    m_redundancyMode = m_redundancyMode & ~(1 << linkId);
    m_RedundantPacketCnt[linkId] = 0;
}

uint32_t
MsduGrouper::GetBAWindowThreshold(uint8_t linkId)
{
    return m_maxAmpduSize[linkId] / 2;
}

bool
MsduGrouper::UpdateAmpduSize(uint8_t linkId, uint32_t size)
{
    if (!m_mode)
        return false;
    if (size)
    {
        // if (m_txopTimeEnd[linkId] == 0)
        //     m_txopNumList[linkId].push_back(size);
    }
    // if (linkId == 0) std::cout << "MAC ADDR: " << m_mac->GetAddress() <<  " m_maxAmpduSize: " <<
    // size << " LinkId: " << uint32_t(linkId) << "最大聚合数：" << m_maxAmpduSize[linkId] <<
    // std::endl;
    if (size > m_maxAmpduSize[linkId])
    {
        m_maxAmpduSize[linkId] = size;
    }
    if (Simulator::Now() > m_startTime + MilliSeconds(100))
    {
        if (size < GetBAWindowThreshold(linkId))
        {       
            m_blockrateList[linkId].push_back((double)size / m_maxAmpduSize[linkId]);
            if (!m_queueStats.blockwindows[linkId].IsStrictlyPositive()) {
                m_queueStats.blockwindows[linkId] = Simulator::Now();
                m_queueStats.m_blockCnt[linkId] += 1;
            }
            if (m_inflighted[1 - linkId])
                m_queueStats.m_blockCnt_tr[linkId] += 1;
        }
        else
        {
            if (m_queueStats.blockwindows[linkId].IsStrictlyPositive())
            {
                m_queueStats.blockwindows_Total[linkId] +=
                    Simulator::Now() - m_queueStats.blockwindows[linkId];
                m_queueStats.blockwindows[linkId] = Seconds(0);
            }
        }
    }

    // if (size == 1)
    //     return false;
    // if (Simulator::Now().GetSeconds() > m_startTime + 1 && size < GetBAWindowThreshold(linkId) *
    // m_redundancyThreshold[linkId] / 100)
    // {
    //     SetRedundancyMode(linkId,  m_redundancyFixedNumber == 0 ? GetBAWindowThreshold(linkId)*
    //     m_redundancyThreshold[linkId] / 100 - size : m_redundancyFixedNumber);
    //     // std::cout << "Redundancy mode open, n = "<< ( m_redundancyFixedNumber == 0 ?
    //     GetBAWindowThreshold(linkId) * m_redundancyThreshold[linkId] / 100 - size :
    //     m_redundancyFixedNumber ) << " on Link " << (uint32_t)linkId << std::endl;
    //     UpdateRedundancyCnt(linkId);
    //     return true; // 开启冗余模式
    // }
    return false;
}

std::unordered_map<std::string, std::vector<uint32_t>>
MsduGrouper::GetNextEdcaParameters()
{
    const auto& params = m_gs->GetNext();
    std::unordered_map<std::string, std::vector<uint32_t>> next_params;
    next_params["No"] = {params.No};
    next_params["CWmins"] = params.CWmins;
    next_params["CWmaxs"] = params.CWmaxs;
    next_params["TxopLimits"] = params.TxopLimits;
    next_params["Aifsns"] = params.Aifsns;
    next_params["AmpduSizes"] = params.AmpduSizes;
    next_params["RTS_CTS"] = params.RTS_CTS;
    next_params["MaxSlrcs"] = params.MaxSlrcs;
    next_params["MaxSsrcs"] = params.MaxSsrcs;
    next_params["RedundancyThresholds"] = params.RedundancyThresholds;
    next_params["RedundancyFixedNumbers"] = params.RedundancyFixedNumbers;
    m_current_params = next_params;

    std::cout << "MaxAmpduSize: " << m_maxAmpduSize[0] << " " << m_maxAmpduSize[1] << std::endl;
    return next_params;
}

std::unordered_map<std::string, std::vector<uint32_t>>
MsduGrouper::GetCurrentEdcaParameters()
{
    return m_current_params;
}

std::unordered_map<std::string, std::vector<uint32_t>>
MsduGrouper::GetNewEdcaParameters()
{
    std::cout << "MaxAmpduSize: " << m_maxAmpduSize[0] << " " << m_maxAmpduSize[1] << std::endl;
    std::unordered_map<std::string, std::vector<uint32_t>> next_params;
    next_params["MaxSlrcs"] = std::vector<uint32_t>{10000, 10000};
    next_params["MaxSsrcs"] = {5, 5};
    return next_params;
}

std::pair<uint8_t, Time>
MsduGrouper::GetNewLinkStates()
{
    return {0b11, MicroSeconds(0)};
}

uint32_t
MsduGrouper::AvailableRedundancy(uint8_t linkId)
{
    if (m_redundancyMode & (1 << linkId))
    {
        if (m_maxRedundantPackets[linkId] > m_RedundantPacketCnt[linkId])
        {
            m_RedundantPacketCnt[linkId] += 1;
            return 1;
        }
        else
        {
            ResetRedundancyMode(linkId);
            return 0;
        }
    }
    return 0;
}

void
MsduGrouper::UpdateRedundancyCnt(uint8_t linkId)
{
    m_RedundantPacketCnt[linkId] = 0;
}

void
MsduGrouper::UpdateRedundancyThreshold(const std::vector<uint32_t> thresholds)
{
    m_redundancyThreshold = thresholds;
}

bool
MsduGrouper::IsGridSearchEnabled()
{
    return m_gs_enable && m_mode;
}

bool
MsduGrouper::IsParamUpdateEnabled()
{
    return m_param_update && m_mode;
}

void
MsduGrouper::UpdateRedundancyFixedNumber(const uint32_t n)
{
    m_redundancyFixedNumber = n;
}

void
MsduGrouper::EnableGridSearch(std::string filename)
{
    m_gs_enable = true;
    m_gs = new GridSearch(filename);
}

void 
MsduGrouper::EnableParamUpdate() {
    m_param_update = true;
}

void
MsduGrouper::SetTxopTimeEnd(uint64_t time /* us */, uint8_t linkId)
{
    m_txopTimeEnd[linkId] = time;
    m_txopTimeList[linkId].push_back(m_txopTimeEnd[linkId] - m_txopTimeBegin[linkId]);
    m_txopList[linkId].push_back(std::make_pair(m_txopTimeBegin[linkId], m_txopTimeEnd[linkId]));
    m_txopNumList[linkId].push_back(linkId);

    m_txopTimeEnd[linkId] = 0;
}

void
MsduGrouper::ResetInflighedCnt()
{
    m_inflighted[0] = 0;
    m_inflighted[1] = 0;
}

std::vector<uint32_t>
MsduGrouper::GetMaxAmpduLength()
{
    return m_maxAmpduSize;
}

std::vector<uint32_t>
MsduGrouper::GetMeanAmpduLength()
{
    uint32_t meanAmpduLength1 =
        std::accumulate(m_txopNumList[0].begin(), m_txopNumList[0].end(), 0) /
        (m_txopNumList[0].size() == 0 ? 1 : m_txopNumList[0].size());
    uint32_t meanAmpduLength2 =
        std::accumulate(m_txopNumList[1].begin(), m_txopNumList[1].end(), 0) /
        (m_txopNumList[1].size() == 0 ? 1 : m_txopNumList[1].size());
    return {meanAmpduLength1, meanAmpduLength2};
}

std::vector<double>
MsduGrouper::GetMeanBlockRate()
{
    double r1 = std::accumulate(m_blockrateList[0].begin(), m_blockrateList[0].end(), 0.0) /
                (m_blockrateList[0].size() == 0 ? 1 : m_blockrateList[0].size());
    double r2 = std::accumulate(m_blockrateList[1].begin(), m_blockrateList[1].end(), 0.0) /
                (m_blockrateList[1].size() == 0 ? 1 : m_blockrateList[1].size());
    return {r1, r2};
}

void
MsduGrouper::ClearStats()
{
    m_txopNumList.clear();
    m_txopList.clear();
    m_txopTimeList.clear();
    m_blockrateList.clear();
    m_maxAmpduSize = {0, 0};
    m_startTime = Simulator::Now();
}

} // namespace ns3