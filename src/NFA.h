#pragma once
#include "methods.h"


class NFA {
 private:
  Machine* automata;
  vector<vector<pair<int, char>>> graph;
  int count;
  std::set<int> finish_vertexes;
  int start;
  vector<vector<pair<int, char>>> mpdka;
  vector<char> sigma = {'a', 'b', 'e'};
 public:
  NFA(string& regularExpression);
  NFA(vector<vector<pair<int, char>>>& graph1, std::set<int>& finish, int startV, int count);
  auto getMPDKA();


  bool check(string expression);
  vector<vector<pair<int, char>>> getGraph();
  std::set<int> getFinish();

  int getStart();
  int getCount();
  void getAutomata();

  string getRegular();
};

auto createNKAfromString();