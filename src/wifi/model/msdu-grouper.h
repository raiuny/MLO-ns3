#ifndef MSDU_GROUPER_H
#define MSDU_GROUPER_H

  
#include "ns3/wifi-mpdu.h"
#include "ns3/wifi-psdu.h"
#include "ns3/wifi-ppdu.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/log.h"
#include "ns3/random-variable-stream.h"
#include "ns3/wifi-mac.h"
#include "ns3/udp-client-server-helper.h"
#include <fstream>
#include <deque>
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
    uint8_t linkId;
};


class QueueStats
{
public:
    QueueStats(Time period, Ptr<WifiMac> mac);
    QueueStats();
    ~QueueStats();
    bool Initialize();
    
    double GetThroughput(uint8_t linkId, Time period);
    /**
     * \brief 这个函数是信道利用率的函数，返回的是信道利用率
     */
    double GetChannelEfficiency(uint8_t linkId, Time period);
    /**
     * \brief 获取指定链路的成功率
     * \param linkId 链路ID
     * \return the success rate in percentage
     */
    double GetMpduSuccessRate(uint8_t linkId, Time period);
    /**
     * \brief 获取指定链路的速率的加权平均，权是PPDU的实际发送时间，值是发送这个PPDU时用的DATA RATE
     * \return the average data rate in mbps
     */
    double GetAverageDataRate(uint8_t linkId, Time period);

    std::vector<double> GetBlockTimeRate(Time period);

    std::vector<uint32_t> GetBlockCnt(Time period);

    std::vector<uint32_t> GetBlockCnt_other_inflight(Time period);


    /**
     * \brief 获取指定链路的最近发送的PPDU的长度
     * \param linkId 链路ID
     * \return 一个包含了最近发的PPDU的长度的vector
     */
    std::vector<uint32_t> GetRecentAMPDULengths(uint8_t linkId, Time period);
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

    Time cycle_time;
    bool m_initialized;
    std::vector<PPDUInfo> m_ppduinfos;
    std::vector<MPDUInfo> m_mpduinfos;
    std::vector<Time> blockwindow_begin;
    std::vector<Time> blockwindow_Total;
    std::map<uint8_t, std::vector<std::pair<Time, Time>>> blockwindow_time;
    std::map<uint8_t, std::vector<Time>> blockwindow_time_other_inflight;
    std::map<std::pair<Mac48Address, uint8_t>, std::vector<WiFiBawQueueIt>> m_bawqueue;
    Ptr<WifiMac> m_mac;
};

struct mldParams {
    uint32_t No{0};
    std::vector<uint32_t> CWmins;
    std::vector<uint32_t> CWmaxs;
    std::vector<uint32_t> Aifsns;
    std::vector<uint32_t> RTS_CTS;
    std::vector<uint32_t> TxopLimits;
    std::vector<uint32_t> AmpduSizes;
    std::vector<uint32_t> MaxSlrcs;
    std::vector<uint32_t> MaxSsrcs;
    std::vector<double> RedundancyThresholds;
    std::vector<uint32_t> RedundancyFixedNumbers;
    void print() const {
        if (this->No == 0) {
            std::cout << "No is 0 \n";
            return;
        }
        std::cout << "No: " << No << "\n";
        printVector("CWmins", CWmins);
        printVector("CWmaxs", CWmaxs);
        printVector("Aifsns", Aifsns);
        printVector("RTS_CTS", RTS_CTS);
        printVector("TxopLimits", TxopLimits);
        printVector("AmpduSizes", AmpduSizes);
        printVector("MaxSlrcs", MaxSlrcs); 
        printVector("MaxSsrcs", MaxSsrcs);  
        printVector("RedundancyThresholds", RedundancyThresholds);
        printVector("RedundancyFixedNumbers", RedundancyFixedNumbers);
    }

    template <typename T>
    void printVector(const std::string& name, const std::vector<T>& vec) const {
        std::cout << name << ": ";
        if (vec.empty()) {
            std::cout << "null";
        } else {
            for (const auto& val : vec) {
                std::cout << val << " ";
            }
        }
        std::cout << "\n";
    }
};

class GridSearch : public Object
{
    public:
        GridSearch (std::string ParamsJsonFilePath = "./scratch/params.json") {
            std::ifstream infile(ParamsJsonFilePath);
            nlohmann::json J;
            infile >> J;
            m_index = 0;
            int i = 0;
            for (const auto & param : J["params"]) {
                std::vector<uint32_t> CWmins = param["CWmins"];
                std::vector<uint32_t> CWmaxs = param["CWmaxs"];
                std::vector<uint32_t> Aifsns = param["Aifsns"];
                std::vector<uint32_t> RTS_CTS = param["RTS/CTS"];
                std::vector<uint32_t> TxopLimits = param["TxopLimits"];
                std::vector<uint32_t> AmpduSize = param["AmpduSize"];
                std::vector<double> RedundancyThresholds = param["RedundancyThreshold"];
                std::vector<uint32_t> RedundancyFixedNumbers = param["RedundancyFixedNumber"];
                std::vector<uint32_t> MaxSlrc = param["MaxSlrc"];
                std::vector<uint32_t> MaxSsrc = param["MaxSsrc"];
                mldParams params;
                params.No = (++i);
                params.CWmins = CWmins;
                params.CWmaxs = CWmaxs;
                params.Aifsns = Aifsns;
                params.RTS_CTS = RTS_CTS;
                params.TxopLimits = TxopLimits;
                params.MaxSlrcs = MaxSlrc;
                params.MaxSsrcs = MaxSsrc;
                params.AmpduSizes = AmpduSize;
                params.RedundancyFixedNumbers = RedundancyFixedNumbers;
                params.RedundancyThresholds = RedundancyThresholds;
                m_combinations.push_back(params);
            }
            m_size = m_combinations.size();
            std::cout << "Size of Params: " << m_size << std::endl;
        }
    
        mldParams GetNext() {
            mldParams & params = m_combinations[m_index];
            m_index = (m_index + 1) % m_combinations.size();
            return params;
        }
    
        uint32_t GetSize() {
            return m_size;
        }

        uint32_t GetIndex() {
            return (m_index + m_combinations.size() - 1) % m_combinations.size();
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
    void AssignAmsduByPr();

    void SetLink1PctByQueuePowAvg(double thp1, double thp2);

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

    void ResetEnqueueNum();

    void SetLink1Pct(double link1Pct);

    double GetLink1Pct();

    uint32_t GetMaxGroupSize();

    Time GetStartTime();
    /**
     * 
     * cwmin, cwmax, txoplimits, aifsns, rts_cts, ampdusize
     */
    mldParams GetNextEdcaParameters();

    mldParams GetCurrentEdcaParameters();

    bool IsGridSearchEnabled();

    bool IsParamUpdateEnabled();

    void UpdateRedundancyThreshold(const std::vector<double> thresholds);

    void UpdateRedundancyFixedNumber(const std::vector<uint32_t> n);

    void EnableGridSearch(std::string filename);
     
    void EnableParamUpdate();

    mldParams GetNewEdcaParameters(bool initial);
    
    std::pair<uint8_t, Time> GetNewLinkStates();

    uint32_t AvailableRedundancy(uint8_t linkId);

    void UpdateRedundancyCnt(uint8_t linkId);

    QueueStats& GetQueueStats() {return m_queueStats;}

    Time GetPeriod() {
        return m_period;
    }

    void ResetInflighedCnt();
    
    std::vector<uint32_t> GetMaxAmpduLength();

    std::vector<double> GetMeanBlockRate(Time period);

    std::vector<uint64_t> GetMeanTxopTime(Time period);

    std::vector<uint32_t> GetMeanTxopMpduNum(Time period);

    void ClearStats();

    friend class QueueStats;
    std::vector<uint32_t> m_maxRedundantPackets; // 最大冗余包数量
    std::vector<uint32_t> m_RedundantPacketCnt;
    std::vector<double> m_redundancyThreshold;
    std::vector<uint32_t> m_redundancyFixedNumber;

    std::unordered_map<uint8_t, std::vector<std::pair<Time, double>>> m_blockrateList; // 卡窗强度统计

    std::unordered_map<uint8_t, uint64_t> m_txopTimeBegin;

    std::unordered_map<uint8_t, uint64_t> m_txopTimeEnd;

    std::unordered_map<uint8_t, std::vector<std::pair<uint64_t, uint64_t>>> m_txopList; // 统计每次txop时间间隔 (txop开始时间, txop结束时间)

    std::unordered_map<uint8_t, std::vector<std::pair<uint64_t, uint32_t>>> m_txopNumList; // 统计每次获得txop时传输mpdu个数

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
    double m_link1Pct; // 分配给链路1的概率
    std::set<WifiContainerQueueId> m_queueIds;
    uint64_t m_enqueueNum;
    std::deque<double> m_historyPct;

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
    bool m_param_update;
    mldParams m_current_params;

};

} // namespace ns3

#endif /* MSDU_GROUPER_H */