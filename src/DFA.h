#pragma once
#include "NFA.h"
#include "methods.h"




class DFA {
 private:
  string reg;
  Machine* automata;
  map<pair<std::set<int>, char>, std::set<int>> new_graph;
  int count;
  std::set<std::set<int>> setOfFinishVertexes;
  std::set<int> start;
  std::set<std::set<int>> RealVertexes;

  vector<vector<pair<int, char>>> mpdka;
  vector<char> sigma = {'a', 'b', 'e'};
  int countMpdka;
  int startMpdka;
  std::set<int> finish_v;
  bool isMPDKA=false;
 public:
  DFA(NFA& automata) {
    vector<vector<pair<int, char>>> graph = automata.getGraph();
    count = automata.getCount();
    std::set<int> finish_vertexes = automata.getFinish();


    auto type_vertexes = compress_components(graph, count, finish_vertexes);
    vector<bool> used1(graph.size(), false);
    BFS(graph, type_vertexes.first.second, type_vertexes.first.first, type_vertexes.second, used1);
    auto result = AlgorithmTompsona(graph, type_vertexes.second, sigma, type_vertexes.first.second);
    auto alreadyInSet = result.first.first;

    auto new_graph_as_map = result.second;
    auto finish_points= result.first.second;
    RealVertexes = alreadyInSet;
    count = static_cast<int>(alreadyInSet.size());
    setOfFinishVertexes = finish_points;
    new_graph = new_graph_as_map;
    start = {type_vertexes.second};

  }
  DFA(string& regularExpression) {
    auto newString = create_postfix_write(regularExpression);
    vector<vector<pair<int, char>>> graph;
    automata = create_machine_from_postfix(newString);
    map<Node*, int> finish_vert;
    count = create_graph(automata, graph, finish_vert);
    std::set<int> finish_vertexes;
    for (auto elem : finish_vert) {
      finish_vertexes.insert(elem.second);
    }
    auto type_vertexes = compress_components(graph, count, finish_vertexes);
    vector<bool> used1(graph.size(), false);
    BFS(graph, type_vertexes.first.second, type_vertexes.first.first, type_vertexes.second, used1);
    auto result = AlgorithmTompsona(graph, type_vertexes.second, sigma, type_vertexes.first.second);
    auto alreadyInSet = result.first.first;

    auto new_graph_as_map = result.second;
    auto finish_points= result.first.second;
    RealVertexes = alreadyInSet;
    count = static_cast<int>(alreadyInSet.size());
    setOfFinishVertexes = finish_points;
    new_graph = new_graph_as_map;
    start = {type_vertexes.second};
  }
  void makeMinimization() {
    addTrash(sigma, new_graph, RealVertexes);
    auto all = minimization(new_graph, sigma, RealVertexes, setOfFinishVertexes, start);
    mpdka = all.first;
    startMpdka = all.second.second;
    finish_v = all.second.first;
    isMPDKA = true;
  }

  void getDFA() {
    if (isMPDKA) {
      for (int i = 0; i < mpdka.size(); ++i) {
        std::cout << i << ": ";
        for (auto v : mpdka[i]) {
          std::cout<<v.first << " " << v.second << "; ";
        }
        std::cout<<'\n';
      }
      for (auto v : finish_v) {
        std::cout << v << " ";
      }
      std::cout << '\n';
      std::cout << startMpdka;
    } else {
      map<std::set<int>, int> rightVertexes;
      int idx = 0;
      for (auto v : RealVertexes) {
        rightVertexes[v] = idx;
        idx += 1;
      }
      for (auto v : RealVertexes) {
        std::cout << rightVertexes[v]<< " ";
        for (auto symbol : sigma) {
          if (symbol != 'e') {
            if (new_graph.contains(pair(v, symbol))) {
              std::cout << rightVertexes[new_graph[pair(v, symbol)]] << " " << symbol << "; ";
            }
          }
        }
        std::cout << '\n';
      }
      for (auto v : setOfFinishVertexes) {
        if (rightVertexes.contains(v)) {
          std::cout << rightVertexes[v] << " ";
        }
      }
      std::cout << '\n';
      std::cout << rightVertexes[start];
    }
  }
  string getRegular() {
    auto copy_graph = new_graph;
    auto copy_sigma = sigma;
    auto copy_RealVertex = RealVertexes;
    addTrash(copy_sigma, copy_graph, copy_RealVertex);
    auto copy_start = start;
    auto copy_setOfFinishVertexes = setOfFinishVertexes;
    auto all = minimization(copy_graph, copy_sigma, copy_RealVertex, copy_setOfFinishVertexes, copy_start);
    return getRegularExpression(all.first, all.second.first, all.second.second);
  }

  bool check(string expression) {
    int idx = 0;
    auto copy_graph = new_graph;
    auto copy_sigma = sigma;
    auto copy_RealVertex = RealVertexes;
    addTrash(copy_sigma, copy_graph, copy_RealVertex);
    auto copy_start = start;
    auto copy_setOfFinishVertexes = setOfFinishVertexes;
    auto allForMPDKA = minimization(copy_graph, copy_sigma, copy_RealVertex, copy_setOfFinishVertexes, copy_start);

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
};

auto createDFAfromString() {
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
  auto automata = NFA(graph, finish_vertex, start, num_vertex);
  return DFA(automata);
}