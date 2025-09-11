#include <iostream>
#include <graph_search2/graph_search2.h>
#include <algorithm>
#include <condition_variable>
#include <vector>
#include <thread>
#include <mutex>

namespace graph_search2{

  bool compareh(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) { return a->hCost() < b->hCost();}
  bool comparegh(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) { return a->gCost()+a->hCost() < b->gCost()+b->hCost();}

  inline void addToOpenList(std::list<std::shared_ptr<Node> >& openList/*ソート済みである*/, std::list<std::shared_ptr<Node> >& newNodes/*破壊的処理される*/, const Param::SolverType& solverType) {
    if(solverType == Param::SolverType::BREADH_FIRST){
      openList.insert(openList.end(), newNodes.begin(), newNodes.end());
    }else if (solverType == Param::SolverType::DEPTH_FIRST){
      openList.insert(openList.begin(), newNodes.begin(), newNodes.end());
    }else if (solverType == Param::SolverType::BEST_FIRST){
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
    }else if (solverType == Param::SolverType::A_STAR){
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

    if(param.threadsNum<=1){
      while(validityNum < param.maxValidityNum){
        if(openList.size()==0) return nullptr;
        std::shared_ptr<Node> target = openList.front();
        if(param.debugLevel >= 2) {
          std::cerr << "openList:" << openList.size() << ", closeList: " << closeList.size() << ", validityNum: " << validityNum << std::endl;
        }
        openList.pop_front();
        if(findNodeInCloseList(closeList,target)) continue;
        validityNum++;
        if(!target->checkValidity()) continue;
        if(target->isGoal()) return target;
        closeList.push_back(target);
        std::list<std::shared_ptr<Node> > children = target->expand();
        addToOpenList(openList, children, param.solverType);
      };
      return nullptr;
    }

    std::shared_ptr<Node> goal = nullptr;
    std::vector<std::unique_ptr<std::thread> > threads;
    bool finished = false; // goal || validityNum >= param.maxValidityNum || (openList.size()==0 && waitingThreadsNum==param.threadsNum)
    for(int i=0;i<param.threadsNum;i++){
      threads.push_back(std::make_unique<std::thread>([&,i]{
        while(true){
          openList_cv.notify_all();
          std::shared_ptr<Node> target;
          {
            std::unique_lock<std::mutex> openList_lock(openList_mtx);
            waitingThreadsNum += 1;
            openList_cv.wait(openList_lock, [&] {
                                              if(openList.size()==0 && waitingThreadsNum==param.threadsNum) finished = true;
                                              return openList.size()!=0 || finished; });
            if(finished) break;
            waitingThreadsNum -= 1;
            target = openList.front();
            openList.pop_front();
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
            goal = target;
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
    return goal;
  }

}
