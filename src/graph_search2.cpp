#include <iostream>
#include <graph_search2/graph_search2.h>
#include <algorithm>
#include <condition_variable>
#include <vector>
#include <queue>
#include <unordered_set>
#include <set>
#include <thread>
#include <mutex>
#include <sys/time.h>

namespace graph_search2{

  struct NodeHash {
    std::size_t operator()(const std::shared_ptr<Node>& u) const {
      return u->hash();
    }
  };
  struct NodeEqual {
    bool operator()(const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) const {
      return lhs->isSame(rhs);
    }
  };

  bool compareh(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
    double aCost = a->hCost();
    double bCost = b->hCost();
    if(aCost > bCost) return true;
    else if(aCost == bCost && a->hash() > b->hash()) return true;
    else return false;
  }
  bool comparegh(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
    double aCost = a->gCost() + a->hCost();
    double bCost = b->gCost() + b->hCost();
    if(aCost > bCost) return true;
    else if(aCost == bCost && a->hash() > b->hash()) return true;
    else return false;
  }
  bool comparewgh(double w, const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
    double aCost = a->gCost() + w * a->hCost();
    double bCost = b->gCost() + w * b->hCost();
    if(aCost > bCost) return true;
    else if(aCost == bCost && a->hash() > b->hash()) return true;
    else return false;
  }

  inline std::shared_ptr<Node> solveWithOutValidity(std::multiset<std::shared_ptr<Node>, decltype(&compareh) > openList, // copy
                                                    std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual > closeList, // copy
                                                    const struct timeval& startTime,
                                                    const Param& param) {
    std::shared_ptr<Node> target_;
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    while(((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) < param.timeout){
      if(openList.size()==0) break;
      std::shared_ptr<Node> target = *(openList.rbegin());
      openList.erase(target); // 降順にソートされているため
      if(closeList.contains(target)) continue;
      if(target->isGoal()) {
        target_ = target;
        break;
      }
      closeList.insert(target);
      std::list<std::shared_ptr<Node> > children = target->expand();
      for(std::shared_ptr<Node>& child: children){
        openList.insert(child);
      }
      gettimeofday(&currentTime, NULL);
    };
    return target_;
  }

  inline std::shared_ptr<Node> popFromOpenListTAMP(std::multiset<std::shared_ptr<Node>, decltype(&compareh) >& openList,
                                                   std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual > closeList,// copy
                                                   std::list<std::shared_ptr<Node> >& guideList,
                                                   const struct timeval& startTime,
                                                   const Param& param) {
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
      std::shared_ptr<Node> target;
      {
        std::multiset<std::shared_ptr<Node>, decltype(&compareh) >::iterator it = openList.find(guide);
        if(it != openList.end()){
          target = *it;
          openList.erase(it);
        }
      }
      if(target) return target;
      guideList.pop_front();
    }
  }

  std::shared_ptr<Node> solveByQueue(const std::list<std::shared_ptr<Node> >& startNodes,
                                     const Param& param) {
    if(param.solverType != Param::SolverType::BREADH_FIRST &&
       param.solverType != Param::SolverType::DEPTH_FIRST){
      std::cerr << "[solveByQueue] Wrong SolverType" << std::endl;
      return nullptr;
    }

    for(std::list<std::shared_ptr<Node> >::const_iterator it=startNodes.begin(); it!=startNodes.end(); it++){
      (*it)->calcCost();
    }

    std::list<std::shared_ptr<Node> > openListQueue;
    std::mutex openList_mtx;
    std::condition_variable openList_cv;
    int waitingThreadsNum = 0;
    for(const std::shared_ptr<Node>& node: startNodes){
      openListQueue.push_back(node);
    }

    std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual > closeList;
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
            ((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) < param.timeout &&
            !param.ptc()){
        if(openListQueue.size()==0) break;
        std::shared_ptr<Node> target = openListQueue.front();
        openListQueue.pop_front();
        if(param.debugLevel >= 2) {
          std::cerr << "openList:" << openListQueue.size() << ", closeList: " << closeList.size() << ", validityNum: " << validityNum << std::endl;
        }
        if(closeList.contains(target)) continue;
        validityNum++;
        if(!target->checkValidity()) continue;
        if(target->isGoal()) {
          target_ = target;
          break;
        }
        closeList.insert(target);
        std::list<std::shared_ptr<Node> > children = target->expand();
        if(param.solverType == Param::SolverType::BREADH_FIRST){
          openListQueue.insert(openListQueue.end(), children.begin(), children.end());
        }else if (param.solverType == Param::SolverType::DEPTH_FIRST){
          openListQueue.insert(openListQueue.begin(), children.begin(), children.end());
        }
      }
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
                                              if(openListQueue.size()==0 && waitingThreadsNum==param.threadsNum) finished = true;
                                              gettimeofday(&currentTime, NULL);
                                              if(((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) > param.timeout) finished = true;
                                              if(param.ptc()) finished = true;
                                              return openListQueue.size()!=0 || finished; });
            if(finished) break;
            waitingThreadsNum -= 1;
            if(openListQueue.size() == 0) continue; // 念の為.
            target = openListQueue.front();
            openListQueue.pop_front();
          }
          if(i==0 && param.debugLevel >= 2) {
            std::cerr << "openList:" << openListQueue.size() << ", closeList: " << closeList.size() << ", validityNum: " << validityNum << std::endl;
          }
          {
            std::lock_guard<std::mutex> closeList_lock(closeList_mtx);
            if(closeList.contains(target)) continue;
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
            if(closeList.contains(target)) continue;
            closeList.insert(target);
          }
          std::list<std::shared_ptr<Node> > children = target->expand();
          {
            std::lock_guard<std::mutex> openList_lock(openList_mtx);
            if(param.solverType == Param::SolverType::BREADH_FIRST){
              openListQueue.insert(openListQueue.end(), children.begin(), children.end());
            }else if (param.solverType == Param::SolverType::DEPTH_FIRST){
              openListQueue.insert(openListQueue.begin(), children.begin(), children.end());
            }
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

  std::shared_ptr<Node> solveByPriorityQueue(const std::list<std::shared_ptr<Node> >& startNodes,
                                             const Param& param) {
    if(param.solverType != Param::SolverType::BEST_FIRST &&
       param.solverType != Param::SolverType::A_STAR){
      std::cerr << "[solveByQueue] Wrong SolverType" << std::endl;
      return nullptr;
    }

    for(std::list<std::shared_ptr<Node> >::const_iterator it=startNodes.begin(); it!=startNodes.end(); it++){
      (*it)->calcCost();
    }

    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node> >, decltype(&compareh) > openListQueue{
      (param.solverType == Param::SolverType::BEST_FIRST) ? compareh :
       ((param.solverType == Param::SolverType::A_STAR) ? comparegh :
        compareh)
    };
    std::mutex openList_mtx;
    std::condition_variable openList_cv;
    int waitingThreadsNum = 0;
    for(const std::shared_ptr<Node>& node: startNodes){
      openListQueue.push(node);
    }

    std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual > closeList;
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
            ((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) < param.timeout &&
            !param.ptc()){
        if(openListQueue.size()==0) break;
        std::shared_ptr<Node>  target = openListQueue.top();
        openListQueue.pop();
        if(param.debugLevel >= 2) {
          std::cerr << "openList:" << openListQueue.size() << ", closeList: " << closeList.size() << ", validityNum: " << validityNum << std::endl;
        }
        if(param.solverType == Param::SolverType::BEST_FIRST){
          if(closeList.contains(target)) continue;
        }else if(param.solverType == Param::SolverType::A_STAR){
          std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >::iterator it = closeList.find(target);
          if(it!=closeList.end() && (*it)->gCost() <= target->gCost()) continue;
        }
        validityNum++;
        if(!target->checkValidity()) continue;
        if(target->isGoal()) {
          target_ = target;
          break;
        }
        if(param.solverType == Param::SolverType::BEST_FIRST){
          closeList.insert(target);
        }else if(param.solverType == Param::SolverType::A_STAR){
          std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >::iterator it = closeList.find(target);
          if(it==closeList.end()) {
            closeList.insert(target);
          }else if((*it)->gCost() <= target->gCost()) {
            continue;
          }else{
            closeList.erase(target);
            closeList.insert(target);
          }
        }

        struct timespec startTime1;
        clock_gettime(CLOCK_MONOTONIC, &startTime1);

        std::list<std::shared_ptr<Node> > children = target->expand();

        for(std::shared_ptr<Node>& child: children){
          openListQueue.push(child);
        }

        gettimeofday(&currentTime, NULL);
      };
      if(param.debugLevel >= 1){
        struct timeval currentTime;
        gettimeofday(&currentTime, NULL);
        std::cerr << "graph_search2 " << (target_ ? "solved" : "failed")<< " in " << (currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6 << std::endl;
      }
      // if(!target_ && openListQueue.size()!=0) target_ = openListQueue.top();
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
                                              if(openListQueue.size()==0 && waitingThreadsNum==param.threadsNum) finished = true;
                                              gettimeofday(&currentTime, NULL);
                                              if(((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) > param.timeout) finished = true;
                                              if(param.ptc()) finished = true;
                                              return openListQueue.size()!=0 || finished; });
            if(finished) break;
            waitingThreadsNum -= 1;
            if(openListQueue.size() == 0) continue; // 念の為.
            target = openListQueue.top();
            openListQueue.pop();
          }
          if(i==0 && param.debugLevel >= 2) {
            std::cerr << "openList:" << openListQueue.size() << ", closeList: " << closeList.size() << ", validityNum: " << validityNum << std::endl;
          }
          {
            std::lock_guard<std::mutex> closeList_lock(closeList_mtx);
            if(param.solverType == Param::SolverType::BEST_FIRST){
              if(closeList.contains(target)) continue;
            }else if(param.solverType == Param::SolverType::A_STAR){
              std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >::iterator it = closeList.find(target);
              if(it!=closeList.end() && (*it)->gCost() <= target->gCost()) continue;
            }
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
            if(param.solverType == Param::SolverType::BEST_FIRST){
              if(closeList.contains(target)) continue;
              closeList.insert(target);
            }else if(param.solverType == Param::SolverType::A_STAR){
              std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >::iterator it = closeList.find(target);
              if(it==closeList.end()) {
                closeList.insert(target);
              }else if((*it)->gCost() <= target->gCost()) {
                continue;
              }else{
                closeList.erase(target);
                closeList.insert(target);
              }
            }
          }
          std::list<std::shared_ptr<Node> > children = target->expand();
          {
            std::lock_guard<std::mutex> openList_lock(openList_mtx);
            for(std::shared_ptr<Node>& child: children){
              openListQueue.push(child);
            }
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

  std::shared_ptr<Node> solveByAnytimePriorityQueueOnce(const std::list<std::shared_ptr<Node> >& startNodes,
                                                        double w,
                                                        double bound,
                                                        const std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >& seenList,
                                                        std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >& closeList,
                                                        struct timeval startTime,
                                                        const Param& param) {

    std::function<bool(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b)> compare = std::bind(comparewgh,w,std::placeholders::_1,std::placeholders::_2);
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node> >, decltype(compare) > openListQueue{compare};
    std::mutex openList_mtx;
    std::condition_variable openList_cv;
    int waitingThreadsNum = 0;
    for(const std::shared_ptr<Node>& node: startNodes){
      openListQueue.push(node);
    }

    closeList.clear();
    std::mutex closeList_mtx;

    unsigned long validityNum = 0;

    if(param.threadsNum<=1){
      std::shared_ptr<Node> target_;
      struct timeval currentTime;
      gettimeofday(&currentTime, NULL);
      std::list<std::shared_ptr<Node> > guideList;
      while(validityNum < param.maxValidityNum &&
            ((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) < param.timeout &&
            !param.ptc()){
        if(openListQueue.size()==0) break;
        std::shared_ptr<Node>  target = openListQueue.top();
        openListQueue.pop();
        if(param.debugLevel >= 2) {
          std::cerr << "openList:" << openListQueue.size() << ", closeList: " << closeList.size() << ", validityNum: " << validityNum << std::endl;
        }
        if(target->gCost() + target->hCost() >= bound) continue;
        {
          std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >::iterator close_it = closeList.find(target);
          std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >::const_iterator seen_it = seenList.find(target);
          if(close_it == closeList.end() && seen_it == seenList.end()){
            // pass
          }else if(seen_it != seenList.end()) {
            if((*seen_it)->gCost() <= target->gCost()) target = *seen_it;
          }else{
            if((*close_it)->gCost() <= target->gCost()) continue;
          }
        }
        validityNum++;
        if(!target->checkValidity()) continue;
        if(target->isGoal()) {
          target_ = target;
          break;
        }
        {
          std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >::iterator it = closeList.find(target);
          if(it==closeList.end()) {
            closeList.insert(target);
          }else if((*it)->gCost() <= target->gCost()) {
            continue;
          }else{
            closeList.erase(target);
            closeList.insert(target);
          }
        }

        struct timespec startTime1;
        clock_gettime(CLOCK_MONOTONIC, &startTime1);

        std::list<std::shared_ptr<Node> > children = target->expand();

        for(std::shared_ptr<Node>& child: children){
          openListQueue.push(child);
        }

        gettimeofday(&currentTime, NULL);
      };
      if(param.debugLevel >= 1){
        struct timeval currentTime;
        gettimeofday(&currentTime, NULL);
        std::cerr << "graph_search2 " << (target_ ? "solved" : "failed")<< " in " << (currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6 <<std::endl;
        if(target_) std::cerr << target_->gCost() << std::endl;
      }
      // if(!target_ && openListQueue.size()!=0) target_ = openListQueue.top();
      return target_;
    }else{

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
                                                if(openListQueue.size()==0 && waitingThreadsNum==param.threadsNum) finished = true;
                                                gettimeofday(&currentTime, NULL);
                                                if(((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) > param.timeout) finished = true;
                                                if(param.ptc()) finished = true;
                                                return openListQueue.size()!=0 || finished; });
              if(finished) break;
              waitingThreadsNum -= 1;
              if(openListQueue.size() == 0) continue; // 念の為.
              target = openListQueue.top();
              openListQueue.pop();
            }
            if(i==0 && param.debugLevel >= 2) {
              std::cerr << "openList:" << openListQueue.size() << ", closeList: " << closeList.size() << ", validityNum: " << validityNum << std::endl;
            }
            if(target->gCost() + target->hCost() >= bound) continue;
            {
              std::lock_guard<std::mutex> closeList_lock(closeList_mtx);
              std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >::iterator close_it = closeList.find(target);
              std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >::const_iterator seen_it = seenList.find(target);
              if(close_it == closeList.end() && seen_it == seenList.end()){
                // pass
              }else if(close_it == closeList.end() && seen_it != seenList.end()) {
                if((*seen_it)->gCost() <= target->gCost()) target = *seen_it;
              }else{
                if((*close_it)->gCost() <= target->gCost()) continue;
              }
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
              std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual >::iterator it = closeList.find(target);
              if(it==closeList.end()) {
                closeList.insert(target);
              }else if((*it)->gCost() <= target->gCost()) {
                continue;
              }else{
                closeList.erase(target);
                closeList.insert(target);
              }
            }
            std::list<std::shared_ptr<Node> > children = target->expand();
            {
              std::lock_guard<std::mutex> openList_lock(openList_mtx);
              for(std::shared_ptr<Node>& child: children){
                openListQueue.push(child);
              }
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

  std::shared_ptr<Node> solveByAnytimePriorityQueue(const std::list<std::shared_ptr<Node> >& startNodes,
                                                    const Param& param) {
    if(param.solverType != Param::SolverType::RWA_STAR){
      std::cerr << "[solveByQueue] Wrong SolverType" << std::endl;
      return nullptr;
    }

    for(std::list<std::shared_ptr<Node> >::const_iterator it=startNodes.begin(); it!=startNodes.end(); it++){
      (*it)->calcCost();
    }

    double w = param.w0;
    double bound = std::numeric_limits<double>::max();
    std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual > seenList;
    std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual > closeList;
    struct timeval startTime;
    gettimeofday(&startTime, NULL);

    std::shared_ptr<Node> result;
    while(true){
      std::shared_ptr<Node> current_result = solveByAnytimePriorityQueueOnce(startNodes, w, bound, seenList, closeList, startTime, param);
      if(!current_result) return result;
      result = current_result;
      if(w==1) return result;
      bound = result->gCost();
      w = std::max(w * param.phi, 1.0);
      for(std::shared_ptr<Node> node: closeList){
        seenList.erase(node);
        seenList.insert(node);
      }
      closeList.clear();
    }
  }

  std::shared_ptr<Node> solveByMultiSet(const std::list<std::shared_ptr<Node> >& startNodes,
                                        const Param& param) {
    if(param.solverType != Param::SolverType::TAMP_BEST_FIRST &&
       param.solverType != Param::SolverType::TAMP_A_STAR){
      std::cerr << "[solveByQueue] Wrong SolverType" << std::endl;
      return nullptr;
    }

    for(std::list<std::shared_ptr<Node> >::const_iterator it=startNodes.begin(); it!=startNodes.end(); it++){
      (*it)->calcCost();
    }

    std::multiset<std::shared_ptr<Node>, decltype(&compareh) > openList{
      (param.solverType == Param::SolverType::TAMP_BEST_FIRST) ? compareh :
        ((param.solverType == Param::SolverType::TAMP_A_STAR) ? comparegh :
         compareh)
        };
    std::mutex openList_mtx;
    std::condition_variable openList_cv;
    int waitingThreadsNum = 0;
    for(const std::shared_ptr<Node>& node: startNodes){
      openList.insert(node);
    }

    std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual > closeList;
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
            ((currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) * 1e-6) < param.timeout &&
            !param.ptc()){
        if(openList.size()==0) break;
        std::shared_ptr<Node> target = popFromOpenListTAMP(openList,
                                                           closeList,
                                                           guideList,
                                                           startTime,
                                                           param);
        if(param.debugLevel >= 2) {
          std::cerr << "openList:" << openList.size() << ", closeList: " << closeList.size() << ", validityNum: " << validityNum << std::endl;
        }
        if(closeList.contains(target)) continue;
        validityNum++;
        if(!target->checkValidity()) continue;
        if(target->isGoal()) {
          target_ = target;
          break;
        }
        closeList.insert(target);
        std::list<std::shared_ptr<Node> > children = target->expand();
        for(std::shared_ptr<Node>& child: children){
          openList.insert(child);
        }
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
                                              if(param.ptc()) finished = true;
                                              return openList.size()!=0 || finished; });
            if(finished) break;
            waitingThreadsNum -= 1;
            if(openList.size() == 0) continue; // 念の為.
            std::lock_guard<std::mutex> closeList_lock(closeList_mtx);
            target = popFromOpenListTAMP(openList,
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
            if(closeList.contains(target)) continue;
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
            closeList.insert(target);
          }
          std::list<std::shared_ptr<Node> > children = target->expand();
          {
            std::lock_guard<std::mutex> openList_lock(openList_mtx);
            for(std::shared_ptr<Node>& child: children){
              openList.insert(child);
            }
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


  std::shared_ptr<Node> solve(const std::list<std::shared_ptr<Node> >& startNodes,
                              const Param& param) {
    if(param.solverType == Param::SolverType::BREADH_FIRST ||
       param.solverType == Param::SolverType::DEPTH_FIRST){
      return solveByQueue(startNodes, param);
    }
    else if(param.solverType == Param::SolverType::BEST_FIRST ||
            param.solverType == Param::SolverType::A_STAR){
      return solveByPriorityQueue(startNodes, param);
    }
    else if(param.solverType == Param::SolverType::TAMP_BEST_FIRST ||
            param.solverType == Param::SolverType::TAMP_A_STAR){
      return solveByMultiSet(startNodes, param);
    } else if(param.solverType == Param::SolverType::RWA_STAR){
      return solveByAnytimePriorityQueue(startNodes, param);
    }else{
      std::cerr << "[solveByQueue] Wrong SolverType" << std::endl;
      return nullptr;
    }
  }

}
