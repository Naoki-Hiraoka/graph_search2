#ifndef GRAPH_SEARCH2_GRAPH_SEARCH2_H
#define GRAPH_SEARCH2_GRAPH_SEARCH2_H

#include <list>
#include <vector>
#include <memory>

namespace graph_search2{
  class Node : public std::enable_shared_from_this<Node> {
  public:

    const std::shared_ptr<Node>& parent() const {return this->parent_;}
    std::shared_ptr<Node>& parent() {return this->parent_;}

    // 各種costを計算する.
    virtual void calcCost() = 0;

    // このnodeからgoalまでの推定コスト
    double hCost() const {return this->hCost_; }

    // start nodeからこのnodeまでのコスト
    double gCost() const {return this->gCost_; }

    // goalに到達しているか
    bool isGoal() const {return this->isGoal_; }

    // このnodeとotherが同じかどうかを判定する. 既に訪れたnodeは再度訪れない
    virtual bool isSame(const std::shared_ptr<Node>& other) const = 0;

    // このnodeの具体的な状態を計算し、実行可能性を評価する.
    virtual bool checkValidity() = 0;

    // 子nodeを生成する. checkValidityを行ったあとに実行すること. 子nodeのcalcCostは行うが、checkValidityは行わない.
    virtual std::list<std::shared_ptr<Node> > expand() = 0;

  protected:
    // 外部から設定される
    std::shared_ptr<Node> parent_;

    // calcCost中に内部で計算される
    double hCost_ = 0;
    double gCost_ = 0;
    bool isGoal_ = false;
  };

  template<typename T>
  std::vector<std::shared_ptr<T> > path(std::shared_ptr<Node> node) {
    if(!node){
      return std::vector<std::shared_ptr<T> >();
    }else if(node->parent()){
      std::vector<std::shared_ptr<T> > p = path<T>(node->parent());
      p.push_back(std::static_pointer_cast<T>(node));
      return p;
    }else{
      return std::vector<std::shared_ptr<T> >{std::static_pointer_cast<T>(node)};
    }
  }

  class Param {
  public:
    enum class SolverType
      {
       BREADH_FIRST,
       DEPTH_FIRST,
       BEST_FIRST,
       A_STAR
      };
    SolverType solverType = SolverType::A_STAR;
    unsigned long maxValidityNum = 1e6;
    unsigned int threadsNum = 1;
    int debugLevel = 0; // 0: no message. 1: time measure. 2: verbose
  };
  std::shared_ptr<Node> solve(const std::list<std::shared_ptr<Node> >& startNodes,
                              const Param& param = Param());

}

#endif
