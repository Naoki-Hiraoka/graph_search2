#include <iostream>
#include <graph_search2/graph_search2.h>
#include <algorithm>
#include <condition_variable>
#include <vector>
#include <thread>
#include <mutex>
#include <sys/time.h>

namespace graph_search2{

  bool compareh(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) { return a->hCost() < b->hCost();}
  bool comparegh(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) { return a->gCost()+a->hCost() < b->gCost()+b->hCost();}

  inline void addToOpenList(std::list<std::shared_ptr<Node> >& openList/*ソート済みである*/, std::list<std::shared_ptr<Node> >& newNodes/*破壊的処理される*/, const Param::SolverType& solverType) {
    if(solverType == Param::SolverType::BREADH_FIRST ||
       solverType == Param::SolverType::TAMP_BREADH_FIRST){
      openList.insert(openList.end(), newNodes.begin(), newNodes.end());
    }else if (solverType == Param::SolverType::DEPTH_FIRST ||
              solverType == Param::SolverType::TAMP_DEPTH_FIRST){
      openList.insert(openList.begin(), newNodes.begin(), newNodes.end());
    }else if (solverType == Param::SolverType::BEST_FIRST ||
              solverType == Param::SolverType::TAMP_BEST_FIRST){
      newNodes.sort(compareh);
      std::list<std::shared_ptr<Node> >::iterator it1 = openList.begin();
      std::list<std::shared_ptr<Node> >::iterator it2 = newNodes.begin();
      while(it2!=newNodes.end()){
        if(it1 == openList.end()){
          openList.insert(openList.end(), it2, newNodes.end());
          it2 = newNodes.end();
        }else if (compareh(*it1,*it2)){
          it1++;
        }else{
          openList.insert(it1,*it2);
          it2++;
        }
      }
    }else if (solverType == Param::SolverType::A_STAR ||
              solverType == Param::SolverType::TAMP_A_STAR){
      newNodes.sort(comparegh);
      std::list<std::shared_ptr<Node> >::iterator it1 = openList.begin();
      std::list<std::shared_ptr<Node> >::iterator it2 = newNodes.begin();
      while(it2!=newNodes.end()){
        if(it1 == openList.end()){
          openList.insert(openList.end(), it2, newNodes.end());
          it2 = newNodes.end();
        }else if (comparegh(*it1,*it2)){
          it1++;
        }else{
          openList.insert(it1,*it2);
          it2++;
        }
      }
    }
  }

  inline bool findNodeInCloseList(const std::vector<std::shared_ptr<Node> >&closeList, const std::shared_ptr<Node>& node){
    for(int i=0;i<closeList.size();i++){
      if(node->isSame(closeList[i])) return true;
    }
    return false;
  }

  inline std::shared_ptr<Node> findAndPopNodeInOpenList(std::list<std::shared_ptr<Node> >&openList, const std::shared_ptr<Node>& node){
    std::list<std::shared_ptr<Node> >::iterator result = std::find_if(openList.begin(),
                                                                      openList.end(),
                                                                      [&](std::shared_ptr<Node>& n){
                                                                        return node->isSame(n);
                                                                      });
    if(result == openList.end()) return nullptr;
    std::shared_ptr<Node> resultNode = *result;
    openList.erase(result);
    return resultNode;
  }

  inline std::shared_ptr<Node> solveWithOutValidity(std::list<std::shared_ptr<Node> > openList, // copy
                                                    std::vector<std::shared_ptr<Node> > closeList, // copy
                                                    const struct timeval& startTime,
                                                    const Param& param) {
    std::shared_ptr<Node> target_;
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    while(((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) < param.timeout){
      if(openList.size()==0) break;
      std::shared_ptr<Node> target = openList.front();
      openList.pop_front();
      if(findNodeInCloseList(closeList,target)) continue;
      if(target->isGoal()) {
        target_ = target;
        break;
      }
      closeList.push_back(target);
      std::list<std::shared_ptr<Node> > children = target->expand();
      addToOpenList(openList, children, param.solverType);
      gettimeofday(&currentTime, NULL);
    };
    return target_;
  }

  inline std::shared_ptr<Node> popFromOpenList(std::list<std::shared_ptr<Node> >& openList,
                                               std::vector<std::shared_ptr<Node> > closeList,// copy
                                               std::list<std::shared_ptr<Node> >& guideList,
                                               const struct timeval& startTime,
                                               const Param& param) {
    if(param.solverType == Param::SolverType::BREADH_FIRST ||
       param.solverType == Param::SolverType::DEPTH_FIRST ||
       param.solverType == Param::SolverType::BEST_FIRST ||
       param.solverType == Param::SolverType::A_STAR){
      std::shared_ptr<Node> target = openList.front();
      openList.pop_front();
      return target;
    }else{
      while(true){
        if(guideList.size() == 0){
          std::shared_ptr<Node> guide = solveWithOutValidity(openList,
                                                             closeList,
                                                             startTime,
                                                             param);
          if(!guide) return nullptr;
          std::vector<std::shared_ptr<Node> > guideList_ = path<Node>(guide);
          guideList = std::list<std::shared_ptr<Node> >(guideList_.begin(),guideList_.end());
        }
        std::shared_ptr<Node> guide = guideList.front();
        guideList.pop_front();
        std::shared_ptr<Node> target = findAndPopNodeInOpenList(openList, guide);
        if(target) return target;
      }
    }
  }

  std::shared_ptr<Node> solve(const std::list<std::shared_ptr<Node> >& startNodes,
                              const Param& param) {
    for(std::list<std::shared_ptr<Node> >::const_iterator it=startNodes.begin(); it!=startNodes.end(); it++){
      (*it)->calcCost();
    }

    std::list<std::shared_ptr<Node> > openList;
    std::mutex openList_mtx;
    std::condition_variable openList_cv;
    int waitingThreadsNum = 0;
    {
      std::list<std::shared_ptr<Node> > tmpStartNodes=startNodes;
      addToOpenList(openList, tmpStartNodes, param.solverType);
    }

    std::vector<std::shared_ptr<Node> > closeList;
    std::mutex closeList_mtx;

    unsigned long validityNum = 0;
    struct timeval startTime;
    gettimeofday(&startTime, NULL);

    if(param.threadsNum<=1){
      std::shared_ptr<Node> target_;
      struct timeval currentTime;
      gettimeofday(&currentTime, NULL);
      std::list<std::shared_ptr<Node> > guideList;
      while(validityNum < param.maxValidityNum &&
            ((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) < param.timeout){
        if(openList.size()==0) break;
        std::shared_ptr<Node> target = popFromOpenList(openList,
                                                       closeList,
                                                       guideList,
                                                       startTime,
                                                       param);
        if(param.debugLevel >= 2) {
          std::cerr << "openList:" << openList.size() << ", closeList: " << closeList.size() << ", validityNum: " << validityNum << std::endl;
        }
        if(findNodeInCloseList(closeList,target)) continue;
        validityNum++;
        if(!target->checkValidity()) continue;
        if(target->isGoal()) {
          target_ = target;
          break;
        }
        closeList.push_back(target);
        std::list<std::shared_ptr<Node> > children = target->expand();
        addToOpenList(openList, children, param.solverType);
        gettimeofday(&currentTime, NULL);
      };
      if(param.debugLevel >= 1){
        struct timeval currentTime;
        gettimeofday(&currentTime, NULL);
        std::cerr << "graph_search2 " << (target_ ? "solved" : "failed")<< " in " << (currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6 << std::endl;
      }
      return target_;
    }

    std::shared_ptr<Node> goal = nullptr;
    std::vector<std::unique_ptr<std::thread> > threads;
    bool finished = false; // goal || validityNum >= param.maxValidityNum || (openList.size()==0 && waitingThreadsNum==param.threadsNum) || (((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) > param.timeout)
    for(int i=0;i<param.threadsNum;i++){
      threads.push_back(std::make_unique<std::thread>([&,i]{
        struct timeval currentTime;
        gettimeofday(&currentTime, NULL);
        std::list<std::shared_ptr<Node> > guideList;
        while(true){
          openList_cv.notify_all();
          std::shared_ptr<Node> target;
          {
            std::unique_lock<std::mutex> openList_lock(openList_mtx);
            waitingThreadsNum += 1;
            openList_cv.wait(openList_lock, [&] {
                                              if(openList.size()==0 && waitingThreadsNum==param.threadsNum) finished = true;
                                              gettimeofday(&currentTime, NULL);
                                              if(((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) > param.timeout) finished = true;
                                              return openList.size()!=0 || finished; });
            if(finished) break;
            waitingThreadsNum -= 1;
            if(openList.size() == 0) continue; // 念の為.
            std::lock_guard<std::mutex> closeList_lock(closeList_mtx);
            target = popFromOpenList(openList,
                                     closeList,
                                     guideList,
                                     startTime,
                                     param);
          }
          if(i==0 && param.debugLevel >= 2) {
            std::cerr << "openList:" << openList.size() << ", closeList: " << closeList.size() << ", validityNum: " << validityNum << std::endl;
          }
          {
            std::lock_guard<std::mutex> closeList_lock(closeList_mtx);
            if(findNodeInCloseList(closeList,target)) continue;
          }
          validityNum++;
          if(validityNum >= param.maxValidityNum) {
            finished = true;
          }
          if(!target->checkValidity()) continue;
          if(target->isGoal()) {
            if(!goal) goal = target;
            finished = true;
            continue;
          }
          {
            std::lock_guard<std::mutex> closeList_lock(closeList_mtx);
            closeList.push_back(target);
          }
          std::list<std::shared_ptr<Node> > children = target->expand();
          {
            std::lock_guard<std::mutex> openList_lock(openList_mtx);
            addToOpenList(openList, children, param.solverType);
          }
        }
        openList_cv.notify_all();
                                                      }));
    }
    for(int i=0;i<threads.size();i++){
      threads[i]->join();
    }
    if(param.debugLevel >= 1){
      struct timeval currentTime;
      gettimeofday(&currentTime, NULL);
      std::cerr << "graph_search2 finished in " << (currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6 << std::endl;
    }
    return goal;
  }

}
