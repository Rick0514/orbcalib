#pragma once

#include "KeyFrame.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "KeyFrameDatabase.h"
#include "G2oTypes.h"
#include "LoopClosing.h"
#include "System.h"

using namespace ORB_SLAM3;

class SubKeyFrameDB : public KeyFrameDatabase
{

public:
    using InvertedFileType = vector<list<KeyFrame*>>;
    
    SubKeyFrameDB() = default;

    const InvertedFileType& getInvertedFile() { return mvInvertedFile; }
    const ORBVocabulary* getVocabulary() { return mpVoc; }
};

class SubLoopClosing : public LoopClosing
{
public:

    SubLoopClosing() : LoopClosing(nullptr, nullptr, nullptr, true, true) {}

    bool SubDetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
        std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
    {
        return DetectCommonRegionsFromLastKF(pCurrentKF, pMatchedKF, gScw, nNumProjMatches, vpMPs, vpMatchedMPs);
    }
};

class CalibC2C
{

private:

    // source camera kfs
    vector<KeyFrame*> srcKFs;
    // target camera database
    SubKeyFrameDB* mpKeyFrameDB;
    ORBmatcher* matcher;
    ORBmatcher* matcherBoW;
    SubLoopClosing* mpLC;

    const float bestCandTh = 0.75f;
    const int bestCandNum = 10;

public:
    CalibC2C(System* src, System* dst);

    vector<KeyFrame*> DetectNBestCandidates(KeyFrame* pKF, int N);

    bool DetectCommonRegionsFromCand(KeyFrame* pKF, vector<KeyFrame*>& vpCand, KeyFrame* &pMatchedKF2, g2o::Sim3 &g2oScw,
        int& bestMatchesReprojNum, vector<MapPoint*> &vpMPs, vector<MapPoint*> &vpMatchedMPs);

    int OptimizeSim3ForCalibr(const vector<KeyFrame*>& vpKF1s, const vector<KeyFrame*>& vpKF2s, vector<vector<MapPoint*>>& vvpMatches, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);

    void RunCalib();

};
