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
  NFA(string regularExpression) {
    auto newString = create_postfix_write(regularExpression);
    automata = create_machine_from_postfix(newString);
    map<Node*, int> finish_vert;
    count = create_graph(automata, graph, finish_vert);
    for (auto elem : finish_vert) {
      finish_vertexes.insert(elem.second);
    }
    start = 0;
  }
  NFA(vector<vector<pair<int, char>>>& graph1, std::set<int>& finish, int startV, int coun): graph(graph1), finish_vertexes(finish), start(startV), count(coun) {}
  auto getMPDKA() {
    auto graph1 = graph;
    int count1 = count;
    auto finish_vertexes1 = finish_vertexes;
    auto type_vertexes = compress_components(graph1, count1, finish_vertexes1);
    vector<bool> used1(graph1.size(), false);
    BFS(graph1, type_vertexes.first.second, type_vertexes.first.first, type_vertexes.second, used1);
    auto result = AlgorithmTompsona(graph1, type_vertexes.second, sigma, type_vertexes.first.second);
    auto alreadyInSet = result.first.first;
    auto new_graph_as_map = result.second;
    auto finish_points= result.first.second;
    addTrash(sigma, new_graph_as_map, alreadyInSet);
    std::set<int> start1 = {type_vertexes.second};
    auto all = minimization(new_graph_as_map, sigma, alreadyInSet, finish_points, start1);
    return all;
  }


  bool check(string expression) {
    int idx = 0;

    auto allForMPDKA = getMPDKA();
    int startMpdka = allForMPDKA.second.second;
    auto finish_set = allForMPDKA.second.first;
    int nowVertex = startMpdka;
    int nextVertex = nowVertex;
    while (idx < expression.size()) {
      nowVertex = nextVertex;
      bool flag = false;
      for (auto ways : allForMPDKA.first[nowVertex]) {
        if (ways.second == expression[idx] && !flag) {
          nextVertex = ways.first;
          idx += 1;
          flag = true;
        }
        if (expression[idx] == 'e' && !flag) {
          idx += 1;
          flag = true;
        }
      }
      if (!flag) {
        return false;
      }
    }
    if (finish_set.contains(nextVertex)) {
      return true;
    }
    return false;
  }
  auto getGraph() {
    return graph;
  }
  auto getFinish() {
    return finish_vertexes;
  }
  int getStart() {
    return start;
  }
  int getCount() {
    return count;
  }
  void getAutomata() {
    for (int i = 0; i < graph.size(); ++i) {
      map<char, std::set<int>> vertexs;
      for (auto v : graph[i]) {
        if (vertexs.contains(v.second)) {
          vertexs[v.second].insert(v.first);
        } else {
          vertexs[v.second] = {v.first};
        }
      }

      std::cout << i << '\n';
      for (auto pairs : vertexs) {
        std::cout << pairs.first << "> : ";
        for (auto elem : pairs.second) {
          std::cout << elem << " ";
        }
        std::cout << '\n';
      }
    }
    for (auto finish : finish_vertexes) {
      std::cout << finish << " ";
    }
  }

  string getRegular() {
    auto all = getMPDKA();
    return getRegularExpression(all.first, all.second.first, all.second.second);
  }
};
NFA createNFAfromString() {
  int num_vertex;
  int finish_num;
  int edges_num;
  int start;
  std::cin >> num_vertex >> finish_num >> edges_num >> start;
  vector<vector<pair<int, char>>> graph(num_vertex);
  std::set<int> finish_vertex;
  for (int i = 0; i < finish_num; ++i) {
    int fin;
    std::cin>>fin;
    finish_vertex.insert(fin);
  }
  for (int i = 0; i < edges_num; ++i) {
    char c;
    int vertex_from;
    int vertex_to;
    std::cin >> vertex_from >> vertex_to >> c;
    graph[vertex_from].push_back(pair(vertex_to, c));
  }
  return NFA(graph, finish_vertex, start, num_vertex);
}





//#include "NFA.h"
//#include "methods.h"
//
//
// NFA::NFA(string& regularExpression) {
//    auto newString = create_postfix_write(regularExpression);
//    automata = create_machine_from_postfix(newString);
//    map<Node*, int> finish_vert;
//    count = create_graph(automata, graph, finish_vert);
//    for (auto elem : finish_vert) {
//      finish_vertexes.insert(elem.second);
//    }
//    start = 0;
//  }
//  NFA::NFA(vector<vector<pair<int, char>>>& graph1, std::set<int>& finish, int startV, int coun): graph(graph1), finish_vertexes(finish), start(startV), count(coun) {}
//  auto NFA::getMPDKA() {
//    auto graph1 = graph;
//    int count1 = count;
//    auto finish_vertexes1 = finish_vertexes;
//    auto type_vertexes = compress_components(graph1, count1, finish_vertexes1);
//    vector<bool> used1(graph1.size(), false);
//    BFS(graph1, type_vertexes.first.second, type_vertexes.first.first, type_vertexes.second, used1);
//    auto result = AlgorithmTompsona(graph1, type_vertexes.second, sigma, type_vertexes.first.second);
//    auto alreadyInSet = result.first.first;
//    auto new_graph_as_map = result.second;
//    auto finish_points= result.first.second;
//    addTrash(sigma, new_graph_as_map, alreadyInSet);
//    std::set<int> start1 = {type_vertexes.second};
//    auto all = minimization(new_graph_as_map, sigma, alreadyInSet, finish_points, start1);
//    return all;
//  }
//
//
//  bool NFA::check(string expression) {
//    int idx = 0;
//
//    auto allForMPDKA = getMPDKA();
//    int startMpdka = allForMPDKA.second.second;
//    auto finish_set = allForMPDKA.second.first;
//    int nowVertex = startMpdka;
//    int nextVertex = nowVertex;
//    while (idx < expression.size()) {
//      nowVertex = nextVertex;
//      bool flag = false;
//      for (auto ways : allForMPDKA.first[nowVertex]) {
//        if (ways.second == expression[idx] && !flag) {
//          nextVertex = ways.first;
//          idx += 1;
//          flag = true;
//        }
//        if (expression[idx] == 'e' && !flag) {
//          idx += 1;
//          flag = true;
//        }
//      }
//      if (!flag) {
//        return false;
//      }
//    }
//    if (finish_set.contains(nextVertex)) {
//      return true;
//    }
//    return false;
//  }
//  vector<vector<pair<int, char>>> NFA::getGraph() {
//    return graph;
//  }
//  std::set<int> NFA::getFinish() {
//    return finish_vertexes;
//  }
//  int NFA::getStart() {
//    return start;
//  }
//  int NFA::getCount() {
//    return count;
//  }
//  void NFA::getAutomata() {
//    for (int i = 0; i < graph.size(); ++i) {
//      map<char, std::set<int>> vertexs;
//      for (auto v : graph[i]) {
//        if (vertexs.contains(v.second)) {
//          vertexs[v.second].insert(v.first);
//        } else {
//          vertexs[v.second] = {v.first};
//        }
//      }
//
//      std::cout << i << '\n';
//      for (auto pairs : vertexs) {
//        std::cout << pairs.first << "> : ";
//        for (auto elem : pairs.second) {
//          std::cout << elem << " ";
//        }
//        std::cout << '\n';
//      }
//    }
//    for (auto finish : finish_vertexes) {
//      std::cout << finish << " ";
//    }
//  }
//
//  string NFA::getRegular() {
//    auto all = getMPDKA();
//    return getRegularExpression(all.first, all.second.first, all.second.second);
//  }
//
//auto createNFAfromString() {
//  int num_vertex;
//  int finish_num;
//  int edges_num;
//  int start;
//  std::cin >> num_vertex >> finish_num >> edges_num >> start;
//  vector<vector<pair<int, char>>> graph(num_vertex);
//  std::set<int> finish_vertex;
//  for (int i = 0; i < finish_num; ++i) {
//    int fin;
//    std::cin>>fin;
//    finish_vertex.insert(fin);
//  }
//  for (int i = 0; i < edges_num; ++i) {
//    char c;
//    int vertex_from;
//    int vertex_to;
//    std::cin >> vertex_from >> vertex_to >> c;
//    graph[vertex_from].push_back(pair(vertex_to, c));
//  }
//  return NFA(graph, finish_vertex, start, num_vertex);
//}
//
//
//
//
