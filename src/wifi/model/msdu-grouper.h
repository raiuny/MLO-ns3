#ifndef MSDU_GROUPER_H
#define MSDU_GROUPER_H

  
#include "ns3/wifi-mpdu.h"
#include "ns3/wifi-psdu.h"
#include "ns3/wifi-ppdu.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/log.h"
#include "ns3/random-variable-stream.h"
#include "ns3/wifi-mac.h"
#include <fstream>
#include "ns3/udp-client-server-helper.h"
#include <range/v3/view/cartesian_product.hpp>
#include <nlohmann/json.hpp>

namespace ns3 {

struct WiFiBawQueueIt
{
    Ptr<const Packet> packet;
    uint32_t seqNo;
    uint8_t retryState;
    uint8_t assignState; //0表示这个msdu未被分配,1表示分配给链路1，2表示分配给链路2，3表示分配给链路1和2
    bool acked;
    bool discarded;
};

struct MPDUInfo
{
    // MPDUInfo() = default;
    // MPDUInfo(const MPDUInfo& other)
    //     : m_Uid(other.m_Uid),
    //       m_receiver(other.m_receiver),
    //       m_size(other.m_size),
    //       m_msduNum(other.m_msduNum),
    //       m_mpduSeqNo(other.m_mpduSeqNo),
    //       m_rxstate(other.m_rxstate),
    //       m_txcount(other.m_txcount),
    //       m_linkIds(other.m_linkIds),
    //       m_txTime(other.m_txTime),
    //       m_ackTime(other.m_ackTime),
    //       DataRate(other.DataRate),
    //       noisePower(other.noisePower),
    //       signalPower(other.signalPower)
    // {}
    
    uint32_t m_Uid;
    Mac48Address m_receiver;
    uint32_t m_size;
    uint32_t m_msduNum;
    uint32_t m_mpduSeqNo;
    bool m_rxstate;//发送成功为true，失败为false
    bool m_txcount;//发送次数
    uint8_t m_linkIds;// 0表示这个mpdu未被分配,1表示分配给链路1，2表示分配给链路2，3表示分配给链路1和2
    Time m_txTime;
    Time m_ackTime;
    double DataRate;
    double noisePower; /*不怎么能拿到，这是接收端的数据*/
    double signalPower;/*不怎么能拿到，这是接收端的数据*/
};

struct PPDUInfo
{
    // std::shared_ptr<WifiMpdu> m_mpdu;
    Time txTime;
    Time txDuration;
    std::vector<MPDUInfo> m_mpduinfos;
};


class QueueStats
{
public:
    QueueStats(Time period);
    QueueStats();
    ~QueueStats();
    bool Initialize();
    /**
     * \return the throughput in mbps
     */
    double GetThroughput();
    double GetThroughput(uint8_t linkId);

    double GetChannelEfficiency();
    /**
     * \brief 这个函数是信道利用率的函数，返回的是信道利用率
     */
    double GetChannelEfficiency(uint8_t linkId);
    double GetMpduSuccessRate();
    /**
     * \brief 获取指定链路的成功率
     * \param linkId 链路ID
     * \return the success rate in percentage
     */
    double GetMpduSuccessRate(uint8_t linkId);
    /**
     * \brief 获取指定链路的速率的加权平均，权是PPDU的实际发送时间，值是发送这个PPDU时用的DATA RATE
     * \return the average data rate in mbps
     */
    double GetAverageDataRate(uint8_t linkId);

    std::vector<double> GetBlockTimeRateAndClear()
    {
        std::vector<double> blockTimeRate;
        blockTimeRate.push_back(blockwindows_Total[0].GetSeconds()/cycle_time.GetSeconds());
        blockTimeRate.push_back(blockwindows_Total[1].GetSeconds()/cycle_time.GetSeconds());
        blockwindows_Total = {Seconds(0), Seconds(0)};
        return blockTimeRate;
    }
    /**
     * \brief 获取指定链路的最近发送的PPDU的长度
     * \param linkId 链路ID
     * \return 一个包含了最近发的PPDU的长度的vector
     */
    std::vector<double> GetRecentAMPDULength(uint8_t linkId);
    /**
     * \brief 获取BawQueue，这个队列我没有按照固定大小来设置，所以不保证大小一定为4096
     * \return 一个包含了BawQueue的map，key是接收者地址和链路ID，value是一个包含了WiFiBawQueueIt的vector
     */
    std::map<std::pair<Mac48Address, uint8_t>, std::vector<WiFiBawQueueIt>>& GetBawQueue() {return m_bawqueue;}
    bool Enqueue(Ptr<const WifiMpdu> mpdu);
    bool Pop(Ptr<const WifiMpdu> mpdu, bool ackordiscard);
    /**
     * \brief 获取PPDU信息，用于精细化计算
     * \return 一个包含了PPDU信息的vector，每个元素是一个PPDUInfo结构体，包含了PPDU的发送时间，发送持续时间，以及发送的MPDU信息
     */
    std::vector<PPDUInfo>& GetPPDUInfos() {return m_ppduinfos;}
    /**
     * \brief 获取MPDU信息，用于精细化计算
     * \return 一个包含了MPDU信息的vector，每个元素是一个MPDUInfo结构体，包含了MPDU的UID，接收者，大小，MSDU数量，发送状态，发送次数，发送链路，发送时间，确认时间，数据速率
     */
    std::vector<MPDUInfo>& GetMPDUInfos() {return m_mpduinfos;}
    std::vector<uint32_t> m_blockCnt;
    std::vector<uint32_t> m_blockCnt_prev;

    std::vector<uint32_t> m_blockCnt_tr;
    std::vector<uint32_t> m_blockCnt_tr_prev;
    Time cycle_time;
    bool m_initialized;
    std::vector<PPDUInfo> m_ppduinfos;
    std::vector<MPDUInfo> m_mpduinfos;
    std::vector<Time> blockwindows;
    std::vector<Time> blockwindows_Total;
    std::map<std::pair<Mac48Address, uint8_t>, std::vector<WiFiBawQueueIt>> m_bawqueue;
};

struct mldParams {
    uint32_t No;
    std::vector<uint32_t> CWmins;
    std::vector<uint32_t> CWmaxs;
    std::vector<uint32_t> Aifsns;
    std::vector<uint32_t> RTS_CTS;
    std::vector<uint32_t> TxopLimits;
    std::vector<uint32_t> AmpduSizes;
    std::vector<uint32_t> MaxSlrcs;
    std::vector<uint32_t> MaxSsrcs;
    std::vector<uint32_t> RedundancyThresholds;
    std::vector<uint32_t> RedundancyFixedNumbers;
};

class GridSearch : public Object
{
    public:
        GridSearch (std::string ParamsJsonFilePath = "./scratch/params.json") {
            std::ifstream infile(ParamsJsonFilePath);
            nlohmann::json J;
            infile >> J;
            std::vector<uint32_t> CWmin = J["CWmin"];
            std::vector<uint32_t> Aifsn = J["Aifsn"];
            std::vector<uint32_t> RTS_CTS = J["RTS/CTS"];
            std::vector<uint32_t> TxopLimit = J["TxopLimits"];
            std::vector<uint32_t> AmpduSize = J["AmpduSize"];
            std::vector<uint32_t> RedundancyThreshold = J["RedundancyThreshold"];
            std::vector<uint32_t> RedundancyFixedNumber = J["RedundancyFixedNumber"];
            std::vector<uint32_t> MaxSlrc = J["MaxSlrc"];
            std::vector<uint32_t> MaxSsrc = J["MaxSsrc"];
            m_index = 0;
            int i = 0;
            mldParams params;
            uint32_t cw1 = CWmin[0];
            params.CWmins = {CWmin[0], CWmin[1]};
            uint32_t cw2 = 1023;
            if (cw1 == 1) cw2 = 3;
            else if (cw1 == 3) cw2 = 7;
            else if (cw1 == 7) cw2 = 15;
            else if (cw1 == 15) cw2 = 1023;
            params.No = (++i);
            params.CWmaxs = {cw2, cw2};
            params.Aifsns = {Aifsn[0], Aifsn[1]};
            params.RTS_CTS = {RTS_CTS[0], RTS_CTS[1]};
            params.TxopLimits = {TxopLimit[0], TxopLimit[1]};
            params.MaxSlrcs = {MaxSlrc[0], MaxSlrc[1]};
            params.MaxSsrcs = {MaxSsrc[0], MaxSsrc[1]};
            params.AmpduSizes = {AmpduSize[0], AmpduSize[1]};
            params.RedundancyFixedNumbers = {RedundancyFixedNumber[0], RedundancyFixedNumber[1]};
            params.RedundancyThresholds = {RedundancyThreshold[0], RedundancyThreshold[1]};
            m_combinations.push_back(params);
            m_size = m_combinations.size();
            std::cout << "Size of Params: " << m_size << std::endl;
        }
    
        const mldParams GetNext() {
            mldParams & params = m_combinations[m_index];
            // std::cout << m_index << ":\n";
            // print(params);
            m_index = (m_index + 1) % m_combinations.size();
            return params;
        }
    
        uint32_t GetSize() {
            return m_size;
        }

        uint32_t GetIndex() {
            return (m_index + m_combinations.size() - 1) % m_combinations.size();
        }

        void print(const mldParams & params) {
            std::cout  << "CWmins: ";
            for(const auto &i : params.CWmins) std::cout  << i << " ";
            std::cout  << "\nCWmaxs: ";
            for(const auto &i : params.CWmaxs) std::cout  << i << " ";
            std::cout  << "\nAifsns: ";
            for(const auto &i : params.Aifsns) std::cout  << i << " ";
            std::cout  << "\nRTS_CTS: ";
            for(const auto &i : params.RTS_CTS) std::cout  << i << " ";
            std::cout  << "\nTxopLimits: ";
            for(const auto &i : params.TxopLimits) std::cout  << i << " ";
            std::cout  << "\nAmpduSizes: ";
            for(const auto &i : params.AmpduSizes) std::cout  << i << " ";
            std::cout  << "\nRedundancyThresholds: ";
            for(const auto &i : params.RedundancyThresholds) std::cout  << i << " ";
            std::cout  << "\nRedundancyFixedNumber: ";
            for(const auto &i : params.RedundancyFixedNumbers) std::cout  << i << " ";
            std::cout  << "\n";
        }
    private:
        std::vector<mldParams> m_combinations;
        uint32_t m_index;
        uint32_t m_size;
};

class MsduGrouper : public Object
{
public:
    static TypeId GetTypeId();
    
    /**
     * 构造函数
     * \param maxGroupSize 每组的最大MSDU数量
     */
    MsduGrouper (uint32_t maxGroupSize, uint32_t maxGroupNumber, Ptr<WifiMacQueue> queue, Ptr<WifiMac> mac, uint32_t mode, Time period);
    MsduGrouper ();
    ~MsduGrouper();
    /**
     * 标记MSDU的组号，聚合Msdu
     * \param msdu 需要标记并聚合的MSDU
     */
    void AggregateMsdu(Ptr<WifiMpdu> msdu);

    /** 
     * 增加当前组号
     */
    void AddCurrentGroup(uint32_t itemgroup);

    /**
     * 模式二分配AMSDU的发送链路
     */
    void AssignAmsdu();

    /**
     * 模式二分配AMSDU的发送链路
     */
    void AssignAmsduByPr();

    void NotifyPacketEnqueue(Ptr<const WifiMpdu> mpdu, bool firstAssignSeqNo);
    void NotifyPhyTxEvent(Ptr<const Packet> packet,
                          uint16_t channelFreqMhz,
                          WifiTxVector txVector,
                          MpduInfo aMpdu,
                          uint16_t staId);
    void NotifyAcked(Ptr<const WifiMpdu> mpdu, uint8_t linkId);
    /**
     * 更新PPDU发送时间
     * \param ppdu PPDU指针
     * \param duration 发送持续时间
     */
    void NotifyPpduTxDuration(Ptr<WifiPpdu const> ppdu, Time duration, uint8_t linkId);

    void NotifyDiscardedMpdu(Ptr<const WifiMpdu> mpdu);

    bool GetRedundancyMode(uint8_t linkId);

    void SetRedundancyMode(uint8_t linkId, uint32_t reNum); // ms

    void ResetRedundancyMode(uint8_t linkId);

    uint32_t GetBAWindowThreshold(uint8_t linkId);

    bool UpdateAmpduSize(uint8_t linkId, uint32_t size);

    void SetTxopTimeEnd(uint64_t time /* ns */, uint8_t linkId);
    /**
     * 
     * cwmin, cwmax, txoplimits, aifsns, rts_cts, ampdusize
     */
    std::unordered_map<std::string, std::vector<uint32_t>> GetNextEdcaParameters();

    std::unordered_map<std::string, std::vector<uint32_t>> GetCurrentEdcaParameters();

    bool IsGridSearchEnabled();

    void UpdateRedundancyThreshold(std::vector<uint32_t> const thresholds);

    void UpdateRedundancyFixedNumber(uint32_t const n);

    void EnableGridSearch(std::string filename);

    std::unordered_map<std::string, std::vector<uint32_t>> GetNewEdcaParameters();
    
    std::pair<uint8_t, Time> GetNewLinkStates();

    uint32_t AvailableRedundancy(uint8_t linkId);

    void UpdateRedundancyCnt(uint8_t linkId);

    QueueStats& GetQueueStats() {return m_queueStats;}

    Time GetPeriod() {
        return m_period;
    }

    void ResetInflighedCnt();
    
    std::vector<uint32_t> GetMaxAmpduLength();

    std::vector<uint32_t> GetMeanAmpduLength();

    std::vector<double> GetMeanBlockRate();

    void ClearStats();

    friend class QueueStats;
    std::vector<uint32_t> m_maxRedundantPackets; // 最大冗余包数量
    std::vector<uint32_t> m_RedundantPacketCnt;
    std::vector<uint32_t> m_redundancyThreshold;
    uint32_t m_redundancyFixedNumber;

    std::unordered_map<uint8_t, std::vector<double>> m_blockrateList; // 卡窗强度统计

    std::unordered_map<uint8_t, std::vector<uint32_t>> m_txopTimeList; // 统计平均txop时间

    std::unordered_map<uint8_t, uint64_t> m_txopTimeBegin;

    std::unordered_map<uint8_t, uint64_t> m_txopTimeEnd;

    std::unordered_map<uint8_t, std::vector<std::pair<uint64_t, uint64_t>>> m_txopList; // 统计每次txop时间间隔

    std::unordered_map<uint8_t, std::vector<uint32_t>> m_txopNumList; // 统计每次获得txop时传输mpdu个数

    std::vector<uint32_t> m_inflighted; // 统计每条链路上正在传输个数

private:
    uint32_t m_maxGroupSize;  // 每组的最大MSDU数量
    Ptr<WifiMacQueue> m_queue;//!< the wifi MAC queue
    Ptr<WifiMac> m_mac; //!< the wifi MAC
    uint32_t m_mode;
    Time m_period;// 统计周期，每次更新只统计当前时间之前period时间内的数据，0代表着统计周期为无穷大

    /* Mode 2 */
    uint32_t m_maxGroupNumber;  // 最大组号 (SN)
    uint32_t m_currentGroup;  // 当前组号
    uint32_t m_currentCount;  // 当前组中的MSDU数量
    Ptr<WifiMpdu> m_firstMsdu;

    /* Mode 1 */
    uint32_t m_redundancyMode;
    std::vector<uint32_t> m_maxAmpduSize;

    Ptr<UniformRandomVariable> m_allocatedLink1Pr; //分配到link1的概率
    Ptr<UniformRandomVariable> m_allocatedLink2Pr; //分配到link2的概率

    QueueStats m_queueStats; 
    Time m_startTime; // 统计起始时间

    /* Gride Search */
    Ptr<GridSearch> m_gs; // 用于周期性搜索最优参数
    bool m_gs_enable; // 是否开启最优参数搜索
    std::unordered_map<std::string, std::vector<uint32_t>> m_current_params;

};

} // namespace ns3

#endif /* MSDU_GROUPER_H */