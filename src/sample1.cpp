#include <graph_search2/graph_search2.h>
#include <iostream>

/*
  -----
  -----
  --x--
  --x--
  s-x-g
 */

class SampleNode : public graph_search2::Node {
public:
  int x = 0;
  int y = 0;

public:
  void calcCost() override{
    this->hCost_ = std::abs(x - 5) + std::abs(y - 0);
    if(this->parent_) this->gCost_ = this->parent_->gCost() + 1;
    else this->gCost_ = 0;
    if(this->x==4 && this->y==0) this->isGoal_ = true;
    else this->isGoal_ = false;
  }

  bool isSame(const std::shared_ptr<graph_search2::Node>& other) const override{
    std::shared_ptr<SampleNode> tmpother = std::static_pointer_cast<SampleNode>(other);
    return this->x == tmpother->x && this->y == tmpother->y;
  }

  bool checkValidity() override{
    //std::cout << "validity " << this->x << " " << this->y << std::endl;

    return (this->x >= 0) && (this->x <= 4) && (this->y >= 0) && (this->y <= 4)
      && !(this->x == 2 && this->y == 0)
      &&  !(this->x == 2 && this->y == 1)
      &&  !(this->x == 2 && this->y == 2);
  }

  std::list<std::shared_ptr<graph_search2::Node> > expand() override{
    //std::cout << "expand " << this->x << " " << this->y << std::endl;

    std::list<std::shared_ptr<graph_search2::Node> > children;
    {
      std::shared_ptr<SampleNode> child = std::make_shared<SampleNode>();
      child->x = this->x+1;
      child->y = this->y;
      child->parent() = this->shared_from_this();
      child->calcCost();
      children.push_back(child);
    }
    {
      std::shared_ptr<SampleNode> child = std::make_shared<SampleNode>();
      child->x = this->x-1;
      child->y = this->y;
      child->parent() = this->shared_from_this();
      child->calcCost();
      children.push_back(child);
    }
    {
      std::shared_ptr<SampleNode> child = std::make_shared<SampleNode>();
      child->x = this->x;
      child->y = this->y+1;
      child->parent() = this->shared_from_this();
      child->calcCost();
      children.push_back(child);
    }
    {
      std::shared_ptr<SampleNode> child = std::make_shared<SampleNode>();
      child->x = this->x;
      child->y = this->y-1;
      child->parent() = this->shared_from_this();
      child->calcCost();
      children.push_back(child);
    }
    return children;
  }
};

int main(){
  std::shared_ptr<SampleNode> startNode = std::make_shared<SampleNode>();
  startNode->x = 0;
  startNode->y = 0;
  startNode->calcCost();
  graph_search2::Param param;
  param.threadsNum = 10;
  //param.debugLevel = 2;

  {
    std::cout << std::endl << "BREADH_FIRST" << std::endl;
    param.solverType = graph_search2::Param::SolverType::BREADH_FIRST;
    std::shared_ptr<graph_search2::Node> result = graph_search2::solve(std::list<std::shared_ptr<graph_search2::Node> >{startNode}, param);
    std::vector<std::shared_ptr<SampleNode> > path = graph_search2::path<SampleNode>(result);
    for(int i=0;i<path.size();i++){
      std::cout << path[i]->x << " " << path[i]->y << std::endl;
    }
  }

  {
    std::cout << std::endl << "DEPTH_FIRST" << std::endl;
    param.solverType = graph_search2::Param::SolverType::DEPTH_FIRST;
    std::shared_ptr<graph_search2::Node> result = graph_search2::solve(std::list<std::shared_ptr<graph_search2::Node> >{startNode}, param);
    std::vector<std::shared_ptr<SampleNode> > path = graph_search2::path<SampleNode>(result);
    for(int i=0;i<path.size();i++){
      std::cout << path[i]->x << " " << path[i]->y << std::endl;
    }
  }

  {
    std::cout << std::endl << "BEST_FIRST" << std::endl;
    param.solverType = graph_search2::Param::SolverType::BEST_FIRST;
    std::shared_ptr<graph_search2::Node> result = graph_search2::solve(std::list<std::shared_ptr<graph_search2::Node> >{startNode}, param);
    std::vector<std::shared_ptr<SampleNode> > path = graph_search2::path<SampleNode>(result);
    for(int i=0;i<path.size();i++){
      std::cout << path[i]->x << " " << path[i]->y << std::endl;
    }
  }

  {
    std::cout << std::endl << "A_STAR" << std::endl;
    param.solverType = graph_search2::Param::SolverType::A_STAR;
    std::shared_ptr<graph_search2::Node> result = graph_search2::solve(std::list<std::shared_ptr<graph_search2::Node> >{startNode}, param);
    std::vector<std::shared_ptr<SampleNode> > path = graph_search2::path<SampleNode>(result);
    for(int i=0;i<path.size();i++){
      std::cout << path[i]->x << " " << path[i]->y << std::endl;
    }
  }

  return 0;
}
