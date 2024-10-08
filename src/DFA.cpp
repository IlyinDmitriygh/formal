#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <stack>
#include <set>
#include <queue>
#include "NFA.h"

using std::vector;
using std::string;
using std::pair;
using std::map;
using std::queue;

// закончили с операциями

// Пишем парсер

/*class NFA {
 private:
  Machine* automata;
  vector<vector<pair<int, char>>> graph;
  int count;
  std::set<int> finish_vertexes;
  int start;
  vector<vector<pair<int, char>>> mpdka;
  vector<char> sigma = {'a', 'b'};
 private:
  string create_postfix_write(string& str) {
    string ans;
    std::stack<char> st;
    std::set<char> set_of_operations;
    set_of_operations = {'$', '*', '+', '^'};

    std::map<char, int> priorities;
    priorities['$'] = 2;
    priorities['*'] = 3;
    priorities['+'] = 1;
    priorities['^'] = 4;

    for (auto elem : str) {
      if (elem == '(') {
        st.push(elem);
      } else if (!set_of_operations.contains(elem) && elem != ')') {
        ans.push_back(elem);
      } else if (elem == ')') {
        char subj = st.top();
        while (subj != '(') {
          ans.push_back(subj);
          st.pop();
          subj = st.top();
        }
        st.pop();
      } else if (set_of_operations.contains(elem)) {
        char next_elem = 'a';
        if (st.size() > 0) {
          next_elem = st.top();
        }
        while (st.size() > 0 && next_elem != '(') {
          if (priorities.contains(next_elem) && priorities[next_elem] < priorities[elem]) {
            break;
          }

          ans.push_back(next_elem);
          st.pop();
          if (st.size() > 0) {
            next_elem = st.top();
          }
        }
        st.push(elem);
      } else {
        ans.push_back(elem);
      }
    }
    while (st.size() > 0) {
      ans.push_back(st.top());
      st.pop();
    }
    return ans;
  }

  Machine* create_machine_from_postfix(string& expression) {
    std::stack<Machine*> st;
    std::set<char> set_of_operations;
    set_of_operations = {'$', '*', '+', '^'};
    for (auto elem : expression) {
      if (set_of_operations.contains(elem)) {
        auto a1 = st.top();
        st.pop();
        Machine* new_machine;
        if (st.size() > 0 && elem != '*' && elem != '^') {
          auto a2 = st.top();
          st.pop();

          if (elem == '$') {
            new_machine = concatenation(*a2, *a1);
          } else if (elem == '+') {
            new_machine = plus(*a1, *a2);
          }
        }
        else {
          if (elem == '*') {
            new_machine = klini_star(*a1);
          } else {
            new_machine = plus_klini(*a1);
          }
        }
        st.push(new_machine);
      } else {
        auto new_machine = new Machine(elem);
        st.push(new_machine);
      }
    }
    std::cout << "size in the end of postfix process : " << st.size() << "\n";

    return st.top();
  }



  void dfs(Node* vertex, map<Node*, int>& used, vector<vector<pair<int, char>>>& graph, int& vertex_num, map<Node*, int>& finish_vert) {
    for (auto son : vertex->sons) {
      if (!used.contains(son.first)) {
        graph[used[vertex]].push_back(pair(vertex_num + 1, son.second));
        used[son.first] = vertex_num + 1;
        graph.push_back(vector<pair<int, char>>());
        if (son.first->is_finish) {
          if (!finish_vert.contains(son.first)) {
            finish_vert[son.first] = vertex_num + 1;
          }
        }
        vertex_num += 1;
        dfs(son.first, used, graph, vertex_num, finish_vert);
      } else {
        graph[used[vertex]].push_back(pair(used[son.first], son.second));
      }
    }
    return;
  }


  int create_graph(Machine* machine, vector<vector<pair<int, char>>>& graph, map<Node*, int>& finish_vert) {
    map<Node*, int> used;
    int vertex_num = 0;
    Node* start = machine->start;
    used[start] = vertex_num;
    graph.push_back(vector<pair<int, char>>());
    dfs(start, used, graph, vertex_num, finish_vert);
    return vertex_num + 1;
  }

  void dfs_vector(int vertex, vector<vector<pair<int, char>>>& graph, vector<int>& tin, vector<pair<int, int>>& tout, vector<bool>& used, int& timer) {
    for (auto son : graph[vertex]) {
      if (!used[son.first] && son.second == 'e') {
        tin[son.first] = timer++;
        used[son.first] = true;
        dfs_vector(son.first, graph, tin, tout, used, timer);
        tout[son.first].second = son.first;
      }
    }
    tout[vertex].first = timer++;
    tout[vertex].second = vertex;
  }

  void dfs_reverse_graph(int vertex, vector<vector<pair<int, char>>>& reverse_graph, vector<bool>& used, vector<int>& components, int color, vector<int>& colors) {
    used[vertex] = true;
    colors[vertex] = color;
    components.push_back(vertex);
    for (auto son : reverse_graph[vertex]) {
      if (!used[son.first] && son.second == 'e') {
        dfs_reverse_graph(son.first, reverse_graph, used, components, color, colors);
      }
    }
  }


  static bool compare_by_tout(std::pair<int, int> a, std::pair<int, int> b) {
    return a.first > b.first;
  }

  pair<pair<map<pair<int, int>, char>, map<int,int>>, int> compress_components(vector<vector<pair<int, char>>>& graph, int counts, std::set<int> finish_vertex) {
    vector<bool> used(counts);
    auto used_reverse = used;
    vector<int> tin(counts);
    vector<pair<int, int>> tout(counts);
    int timer = 0;
    tout[0].second = 0;
    tin[0] = timer++;

    for (int i = 0; i < counts; ++i) {
      if (!used[i]) {
        dfs_vector(i, graph, tin, tout, used, timer);

      }
    }
    std::sort(tout.begin(), tout.end(), compare_by_tout);

    std::vector<vector<pair<int, char>>> reverse_graph(counts);
    for (int i = 0; i < counts; ++i) {
      auto v1 = graph[i];
      for (auto edge : v1) {
        reverse_graph[edge.first].push_back(pair(i, edge.second));
      }
    }
    vector<vector<int>> components(0);
    vector<int> colors(counts);
    int color = 0;
    int idx = 0;

    for (int i = 0; i < counts; ++i) {
      auto pair = tout[i];
      int vertex = pair.second;
      if (!used_reverse[vertex]) {
        components.push_back(vector<int>());
        dfs_reverse_graph(vertex, reverse_graph, used_reverse, components[idx], color, colors);
        color += 1;
        idx += 1;
      }
    }



    std::set<int> set_finish_vertex = finish_vertex;
    map<int, int> new_finish_vertex;
    int new_start_vertex = colors[0];
    map<pair<pair<int, int>, char>, int> new_edges;
    map<pair<int, int>, char> new_epsilon_edges;
    vector<vector<pair<int, char>>> new_graph(color);
    for (auto& component : components) {
      if (!component.empty()) {
        for (auto vertex : component) {
          int color_from = colors[vertex];
          if (set_finish_vertex.contains(vertex) && !new_finish_vertex.contains(colors[vertex])) {
            new_finish_vertex[colors[vertex]] = 1;
          }
          for (auto vertex_to : graph[vertex]) {
            int color_to = colors[vertex_to.first];
            if (!new_edges.contains(pair(pair(color_from, color_to), vertex_to.second))) {
              new_edges[pair(pair(color_from, color_to), vertex_to.second)] = 1;
              if (vertex_to.second == 'e' && !new_epsilon_edges.contains(pair(color_from, color_to))) {
                new_epsilon_edges[pair(color_from, color_to)] = 'e';

              }
              new_graph[color_from].push_back(pair(color_to, vertex_to.second));
            }
          }
        }
      }
    }
    int index = 0;

    for (auto v : new_graph) {
      std::cout << index << " : ";
      for (auto u : v) {
        std::cout << "( " << u.first << " " << u.second << ")" << " ";
      }
      std::cout << '\n';
      index += 1;
    }
    for (auto v : new_finish_vertex) {
      std::cout << v.first << " ";
    }
    std::cout << new_start_vertex;

    graph = new_graph;
    return pair(pair(new_epsilon_edges, new_finish_vertex), new_start_vertex);
  }

  void create_new_transitions(vector<vector<pair<int, char>>>& graph, map<pair<int, int>, char>& epsilon_transitions, int vertex, map<int, int>& finish_vertexes) {
    auto copy = graph[vertex];
    auto copy_all = graph;
    std::set<pair<pair<int, int>, char>> new_transitions;
    for (auto son : copy) {
      new_transitions.insert(pair(pair(vertex, son.first), son.second));
    }
    int difference = 0;
    for (int i = 0; i < static_cast<int>(copy.size()); ++i) {
      auto son = copy[i];
      if (son.second == 'e') {
        if (!graph[son.first].empty()) {
          for (auto grandson: copy_all[son.first]) {
            if (!new_transitions.contains(pair(pair(vertex, grandson.first), grandson.second))) {
              graph[vertex].push_back(pair(grandson.first, grandson.second));
              new_transitions.insert(pair(pair(vertex, grandson.first), grandson.second));
            }
            if (grandson.second == 'e' && !epsilon_transitions.contains(pair(vertex, grandson.first))) {
              epsilon_transitions[pair(vertex, grandson.first)] = 'e';
            }
          }
        }
        if (finish_vertexes.contains(son.first)) {
          finish_vertexes[vertex] = 1;
        }
        epsilon_transitions.erase(pair(vertex, son.first));
        graph[vertex].erase(graph[vertex].begin() + i - difference);
        difference += 1;
      }
    }
  }

  void BFS(vector<vector<pair<int, char>>>& graph,
           map<int, int>& finish_vertexes,
           map<pair<int, int>, char>& epsilon_transitions,
           int start, vector<bool>& used1) {


    const int KMagic = 100000000;
    int prev_count_unused = 0;
    int count_unused = KMagic;
    while (!epsilon_transitions.empty() && count_unused != prev_count_unused) {
      queue<int> queue;
      queue.push(start);
      vector<bool> used(used1.size(), false);
      while (!queue.empty()) {
        int v = queue.front();
        used[v] = true;
        create_new_transitions(graph, epsilon_transitions, v, finish_vertexes);
        for (auto son: graph[v]) {
          if (!used[son.first]) {
            queue.push(son.first);
          }
        }
        queue.pop();
      }
      prev_count_unused = count_unused;
      count_unused = 0;
      for (auto v : used) {
        count_unused += 1 - static_cast<int>(v);
      }
      std::cout<<"ВСего осталось " << epsilon_transitions.size() << '\n';
      used1 = used;
    }
    for (int i = 0; i < static_cast<int>(used1.size()); ++i) {
      if (!used1[i]) {
        graph[i].clear();
        if (finish_vertexes.contains(i)) {
          finish_vertexes.erase(i);
        }
      }
    }
  }

  auto AlgorithmTompsona(vector<vector<pair<int, char>>>& graph, int start, vector<char>& sigma, map<int, int>& finish_vertexes) {
    std::set<std::set<int>> finish_vert;
    map<pair<std::set<int>, char>, std::set<int>> new_graph;
    for (int i = 0; i < static_cast<int>(graph.size()); ++i) {
      std::set<int> new_v = {i};
      if (finish_vertexes.contains(i)) {
        finish_vert.insert(new_v);
      }
    }
    for (int i = 0; i < static_cast<int>(graph.size()); ++i) {
      std::set<int> new_v = {i};
      for (auto elem : sigma) {
        bool flag = false;
        std::set<int> new_son;
        for (auto son: graph[i]) {
          if (son.second == elem) {
            new_son.insert(son.first);
            if (finish_vertexes.contains(son.first)) {
              flag = true;
            }
          }
        }
        if (!new_graph.contains(pair(new_v, elem)) && !new_son.empty()) {
          new_graph[pair(new_v, elem)] = new_son;

        }
        if (!finish_vert.contains(new_son) && flag) {
          finish_vert.insert(new_son);
        }
      }
    }
    std::queue<std::set<int>> query;
    std::set<int> st = {start};
    query.push(st);

    std::set<std::set<int>> alreadyWasInSet = {st};
    while (!query.empty()) {
      auto q = query.front();
      query.pop();
      for (auto elem : sigma) {
        std::set<int> new_vertex;
        bool flag = false;
        if (new_graph.contains(pair(q, elem))) {
          for (auto v : new_graph[pair(q, elem)]) {
            std::set<int> v_set = {v};
            if (finish_vert.contains(v_set) && !flag) {
              flag = true;
            }
          }
          for (auto v : new_graph[pair(q, elem)]) {
            new_vertex.insert(v);
          }
        }
        if (!alreadyWasInSet.contains(new_vertex) && !new_vertex.empty()) {
          for (auto symbol : sigma) {
            std::set<int> to_vertex;
            for (auto v : new_vertex) {
              std::set<int> other = {v};
              if (new_graph.contains(pair(other, symbol))) {
                for (auto u: new_graph[pair(other, symbol)]) {
                  to_vertex.insert(u);
                }
              }
            }
            if (!to_vertex.empty()) {
              new_graph[pair(new_vertex, symbol)] = to_vertex;
            }
          }
          alreadyWasInSet.insert(new_vertex);
          query.push(new_vertex);
          if (flag) {
            finish_vert.insert(new_vertex);
          }
        }
      }
    }
    // вернем только те вершины, которые были в очереди и map с переходами и новые финишные сосстояния

    return pair(pair(alreadyWasInSet, finish_vert), new_graph);

  }

  void addTrash(vector<char>& sigma, map<pair<std::set<int>, char>, std::set<int>>& graph, std::set<std::set<int>>& alreadyWasInSet) {
    std::set<int> Trash = {-1};
    bool flag = false;
    for (auto v : alreadyWasInSet) {
      for (auto symbol : sigma) {
        if (symbol != 'e') {
          if (!graph.contains(pair(v, symbol))) {
            flag = true;
            graph[pair(v, symbol)] = Trash;
          }
        }
      }
    }
    if (flag) {
      for (auto elem: sigma) {
        if (elem != 'e') {
          graph[pair(Trash, elem)] = Trash;
        }
      }
      alreadyWasInSet.insert(Trash);
    }
  }

  auto minimization(map<pair<std::set<int>, char>, std::set<int>>& graph, vector<char>& sigma, std::set<std::set<int>>& alreadyWasInSet, std::set<std::set<int>>& finish_vert, std::set<int> start) {
    map<std::set<int>, int> start_map;
    for (auto v : alreadyWasInSet) {
      if (finish_vert.contains(v)) {
        start_map[v] = 1;
      } else {
        start_map[v] = 0;
      }
    }
    int count_class_prev = -10;
    int count_class = 0;
    map<std::set<int>, int> mapOfClass;
    while (count_class_prev != count_class) {
      count_class_prev = count_class;
      count_class = 0;
      map<vector<std::set<int>>, int> mapOfNewVertexes;
      map<std::set<int>, int> mapOfNewClasses;
      map<vector<int>, int> same_transition;
      vector<vector<std::set<int>>> vertexes_by_classes(alreadyWasInSet.size() + 1);
      for (auto v : alreadyWasInSet) {
        vector<int> transitions;
        for (auto symbol : sigma) {
          if (graph.contains(pair(v, symbol))) {
            auto vertex_in_transition = graph[pair(v, symbol)];
            int old_class_equal = start_map[vertex_in_transition];
            transitions.push_back(old_class_equal);
          }
        }
        if (same_transition.contains(transitions)) {
          bool flag = false;
          for (auto v_candidate : vertexes_by_classes[same_transition[transitions]]) {
            if (start_map[v] == start_map[v_candidate] && !flag) {
              mapOfNewClasses[v] = mapOfNewClasses[v_candidate];
              vertexes_by_classes[same_transition[transitions]].push_back(v);
              flag = true;
            }
          }
          if (!flag) {
            vertexes_by_classes[same_transition[transitions]].push_back(v);
            count_class+=1;
            mapOfNewClasses[v] = count_class;
          }
        } else {
          count_class += 1;
          same_transition[transitions] = count_class;
          vertexes_by_classes[same_transition[transitions]].push_back(v);
          mapOfNewClasses[v] = count_class;
        }
      }
      start_map = mapOfNewClasses;
    }

    int colors = count_class;
    std::set<int> finish_v;
    map<pair<int, char>, int> newEdges;
    vector<vector<pair<int, char>>> Mpdka(colors + 1);
    int start_color = 0;
    for (auto v : start_map) {
      auto vertex = v.first;
      int color = v.second;
      if (vertex == start) {
        start_color = color;
      }
      if (finish_vert.contains(vertex)) {
        finish_v.insert(color);
      }
      for (auto symbol : sigma) {
        if (symbol != 'e') {
          auto vertex_to = graph[pair(vertex, symbol)];
          int color_to = start_map[vertex_to];
          if (!newEdges.contains(pair(color, symbol))) {
            newEdges[pair(color, symbol)] = color_to;
            Mpdka[color].push_back(pair(color_to, symbol));
          }
        }
      }
    }
    return pair(Mpdka, pair(finish_v, start_color));
  }

  auto getRegularExpression(vector<vector<pair<int, char>>> Automata, std::set<int>& finish_vertexes, int start) {
    Automata.push_back(vector<pair<int, char>>());
    int sizeOfAutomata = static_cast<int>(Automata.size());
    for (auto finish_v : finish_vertexes) {
      Automata[finish_v].push_back(pair(sizeOfAutomata - 1, 'e'));
    }

    vector<map<int, string>> automataMap(Automata.size());
    vector<map<int, string>> reverseAutomata(Automata.size());


    for (int i = 1; i < sizeOfAutomata; ++i) {
      for (auto v : Automata[i]) {
        if (automataMap[i].contains(v.first)) {
          automataMap[i][v.first] = automataMap[i][v.first] + "+" + (v.second);
          reverseAutomata[v.first][i] = reverseAutomata[v.first][i] + "+" + (v.second);
        } else {
          automataMap[i][v.first] = (v.second);
          reverseAutomata[v.first][i] = (v.second);
        }
      }
    }
    std::set<int> vertexesAll;
    int count = static_cast<int>(automataMap.size());
    for (int i = 1; i < Automata.size() - 1; ++i) {
      vertexesAll.insert(i);
    }

    const int kMagic = 10000000;
    while (count > 3) {
      int idxDelete = start;
      int minDeg = kMagic;
      for (auto i : vertexesAll) {
        int nowDeg;
        if (i != start) {
          nowDeg = static_cast<int>(reverseAutomata[i].size() * Automata[i].size());
          if (nowDeg < minDeg) {
            minDeg = nowDeg;
            idxDelete = i;
          }
        }
      }
      if (idxDelete != start) {
        string klini = "";
        if (automataMap[idxDelete].contains(idxDelete)) {
          klini = "(" + automataMap[idxDelete][idxDelete] + ")" + "*";
          automataMap[idxDelete].erase(idxDelete);
          reverseAutomata[idxDelete].erase(idxDelete);
        }
        for (auto& v_reverse : reverseAutomata[idxDelete]) {
          for (auto& u : automataMap[idxDelete]) {
            if (automataMap[v_reverse.first].contains(u.first)) {
              if (automataMap[v_reverse.first][u.first] != v_reverse.second + klini +  u.second) {
                automataMap[v_reverse.first][u.first] = automataMap[v_reverse.first][u.first] + "+" +
                                                        v_reverse.second + klini + u.second;
                reverseAutomata[u.first][v_reverse.first] = reverseAutomata[u.first][v_reverse.first] + "+" +
                                                            v_reverse.second + klini + u.second;
              }
            } else {
              automataMap[v_reverse.first][u.first] = v_reverse.second + klini + u.second;
              reverseAutomata[u.first][v_reverse.first] = v_reverse.second + klini + u.second;
            }
          }
        }
        for (auto v_rev : reverseAutomata[idxDelete]) {
          if (v_rev.first != idxDelete) {
            automataMap[v_rev.first].erase(idxDelete);
          }
        }

        for (auto u : automataMap[idxDelete]) {
          if (u.first != idxDelete) {
            reverseAutomata[u.first].erase(idxDelete);
          }
        }
        automataMap[idxDelete].clear();
        reverseAutomata[idxDelete].clear();
        vertexesAll.erase(idxDelete);
        count -= 1;
      }
      std::cout << automataMap.size() << '\n';
    }

    string alpha = "e";
    string beta = "e";
    string gamma = "e";
    string delta = "e";
    if (automataMap[start].contains(start)) {
      alpha = automataMap[start][start];
    }
    for (auto v : automataMap[start]) {
      if (v.first != start) {
        beta = v.second;
        if (automataMap[v.first].contains(v.first)) {
          gamma = automataMap[v.first][v.first];
        }
        if (automataMap[v.first].contains(start)) {
          delta = automataMap[v.first][start];
        }
      }
    }
    string ans = "(" + alpha + ")*" + "("+beta+")" + "("+ gamma + "+" + delta + "("+ alpha + ")"+ "*" + beta + ")" + "*";
    return ans;
  }

 public:
  NFA(string& regularExpression) {
    auto newString = create_postfix_write(regularExpression);
    automata = create_machine_from_postfix(newString);
    map<Node*, int> finish_vert;
    count = create_graph(automata, graph, finish_vert);
    for (auto elem : finish_vert) {
      finish_vertexes.insert(elem.second);
    }
    start = 0;
  }

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


  bool check(string& expression) {
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
};*/

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

 private:
  string create_postfix_write(string& str) {
    string ans;
    std::stack<char> st;
    std::set<char> set_of_operations;
    set_of_operations = {'$', '*', '+', '^'};

    std::map<char, int> priorities;
    priorities['$'] = 2;
    priorities['*'] = 3;
    priorities['+'] = 1;
    priorities['^'] = 4;

    for (auto elem : str) {
      if (elem == '(') {
        st.push(elem);
      } else if (!set_of_operations.contains(elem) && elem != ')') {
        ans.push_back(elem);
      } else if (elem == ')') {
        char subj = st.top();
        while (subj != '(') {
          ans.push_back(subj);
          st.pop();
          subj = st.top();
        }
        st.pop();
      } else if (set_of_operations.contains(elem)) {
        char next_elem = 'a';
        if (st.size() > 0) {
          next_elem = st.top();
        }
        while (st.size() > 0 && next_elem != '(') {
          if (priorities.contains(next_elem) && priorities[next_elem] < priorities[elem]) {
            break;
          }

          ans.push_back(next_elem);
          st.pop();
          if (st.size() > 0) {
            next_elem = st.top();
          }
        }
        st.push(elem);
      } else {
        ans.push_back(elem);
      }
    }
    while (st.size() > 0) {
      ans.push_back(st.top());
      st.pop();
    }
    return ans;
  }

  Machine* create_machine_from_postfix(string& expression) {
    std::stack<Machine*> st;
    std::set<char> set_of_operations;
    set_of_operations = {'$', '*', '+', '^'};
    for (auto elem : expression) {
      if (set_of_operations.contains(elem)) {
        auto a1 = st.top();
        st.pop();
        Machine* new_machine;
        if (st.size() > 0 && elem != '*' && elem != '^') {
          auto a2 = st.top();
          st.pop();

          if (elem == '$') {
            new_machine = concatenation(*a2, *a1);
          } else if (elem == '+') {
            new_machine = plus(*a1, *a2);
          }
        }
        else {
          if (elem == '*') {
            new_machine = klini_star(*a1);
          } else {
            new_machine = plus_klini(*a1);
          }
        }
        st.push(new_machine);
      } else {
        auto new_machine = new Machine(elem);
        st.push(new_machine);
      }
    }
    std::cout << "size in the end of postfix process : " << st.size() << "\n";

    return st.top();
  }



  void dfs(Node* vertex, map<Node*, int>& used, vector<vector<pair<int, char>>>& graph, int& vertex_num, map<Node*, int>& finish_vert) {
    for (auto son : vertex->sons) {
      if (!used.contains(son.first)) {
        graph[used[vertex]].push_back(pair(vertex_num + 1, son.second));
        used[son.first] = vertex_num + 1;
        graph.push_back(vector<pair<int, char>>());
        if (son.first->is_finish) {
          if (!finish_vert.contains(son.first)) {
            finish_vert[son.first] = vertex_num + 1;
          }
        }
        vertex_num += 1;
        dfs(son.first, used, graph, vertex_num, finish_vert);
      } else {
        graph[used[vertex]].push_back(pair(used[son.first], son.second));
      }
    }
    return;
  }


  int create_graph(Machine* machine, vector<vector<pair<int, char>>>& graph, map<Node*, int>& finish_vert) {
    map<Node*, int> used;
    int vertex_num = 0;
    Node* start = machine->start;
    used[start] = vertex_num;
    graph.push_back(vector<pair<int, char>>());
    dfs(start, used, graph, vertex_num, finish_vert);
    return vertex_num + 1;
  }

  void dfs_vector(int vertex, vector<vector<pair<int, char>>>& graph, vector<int>& tin, vector<pair<int, int>>& tout, vector<bool>& used, int& timer) {
    for (auto son : graph[vertex]) {
      if (!used[son.first] && son.second == 'e') {
        tin[son.first] = timer++;
        used[son.first] = true;
        dfs_vector(son.first, graph, tin, tout, used, timer);
        tout[son.first].second = son.first;
      }
    }
    tout[vertex].first = timer++;
    tout[vertex].second = vertex;
  }

  void dfs_reverse_graph(int vertex, vector<vector<pair<int, char>>>& reverse_graph, vector<bool>& used, vector<int>& components, int color, vector<int>& colors) {
    used[vertex] = true;
    colors[vertex] = color;
    components.push_back(vertex);
    for (auto son : reverse_graph[vertex]) {
      if (!used[son.first] && son.second == 'e') {
        dfs_reverse_graph(son.first, reverse_graph, used, components, color, colors);
      }
    }
  }


  static bool compare_by_tout(std::pair<int, int> a, std::pair<int, int> b) {
    return a.first > b.first;
  }

  pair<pair<map<pair<int, int>, char>, map<int,int>>, int> compress_components(vector<vector<pair<int, char>>>& graph, int counts, std::set<int> finish_vertex) {
    vector<bool> used(counts);
    auto used_reverse = used;
    vector<int> tin(counts);
    vector<pair<int, int>> tout(counts);
    int timer = 0;
    tout[0].second = 0;
    tin[0] = timer++;

    for (int i = 0; i < counts; ++i) {
      if (!used[i]) {
        dfs_vector(i, graph, tin, tout, used, timer);

      }
    }
    std::sort(tout.begin(), tout.end(), compare_by_tout);

    std::vector<vector<pair<int, char>>> reverse_graph(counts);
    for (int i = 0; i < counts; ++i) {
      auto v1 = graph[i];
      for (auto edge : v1) {
        reverse_graph[edge.first].push_back(pair(i, edge.second));
      }
    }
    vector<vector<int>> components(0);
    vector<int> colors(counts);
    int color = 0;
    int idx = 0;

    for (int i = 0; i < counts; ++i) {
      auto pair = tout[i];
      int vertex = pair.second;
      if (!used_reverse[vertex]) {
        components.push_back(vector<int>());
        dfs_reverse_graph(vertex, reverse_graph, used_reverse, components[idx], color, colors);
        color += 1;
        idx += 1;
      }
    }



    std::set<int> set_finish_vertex = finish_vertex;
    map<int, int> new_finish_vertex;
    int new_start_vertex = colors[0];
    map<pair<pair<int, int>, char>, int> new_edges;
    map<pair<int, int>, char> new_epsilon_edges;
    vector<vector<pair<int, char>>> new_graph(color);
    for (auto& component : components) {
      if (!component.empty()) {
        for (auto vertex : component) {
          int color_from = colors[vertex];
          if (set_finish_vertex.contains(vertex) && !new_finish_vertex.contains(colors[vertex])) {
            new_finish_vertex[colors[vertex]] = 1;
          }
          for (auto vertex_to : graph[vertex]) {
            int color_to = colors[vertex_to.first];
            if (!new_edges.contains(pair(pair(color_from, color_to), vertex_to.second))) {
              new_edges[pair(pair(color_from, color_to), vertex_to.second)] = 1;
              if (vertex_to.second == 'e' && !new_epsilon_edges.contains(pair(color_from, color_to))) {
                new_epsilon_edges[pair(color_from, color_to)] = 'e';

              }
              new_graph[color_from].push_back(pair(color_to, vertex_to.second));
            }
          }
        }
      }
    }
    int index = 0;

    for (auto v : new_graph) {
      std::cout << index << " : ";
      for (auto u : v) {
        std::cout << "( " << u.first << " " << u.second << ")" << " ";
      }
      std::cout << '\n';
      index += 1;
    }
    for (auto v : new_finish_vertex) {
      std::cout << v.first << " ";
    }
    std::cout << new_start_vertex;

    graph = new_graph;
    return pair(pair(new_epsilon_edges, new_finish_vertex), new_start_vertex);
  }

  void create_new_transitions(vector<vector<pair<int, char>>>& graph, map<pair<int, int>, char>& epsilon_transitions, int vertex, map<int, int>& finish_vertexes) {
    auto copy = graph[vertex];
    auto copy_all = graph;
    std::set<pair<pair<int, int>, char>> new_transitions;
    for (auto son : copy) {
      new_transitions.insert(pair(pair(vertex, son.first), son.second));
    }
    int difference = 0;
    for (int i = 0; i < static_cast<int>(copy.size()); ++i) {
      auto son = copy[i];
      if (son.second == 'e') {
        if (!graph[son.first].empty()) {
          for (auto grandson: copy_all[son.first]) {
            if (!new_transitions.contains(pair(pair(vertex, grandson.first), grandson.second))) {
              graph[vertex].push_back(pair(grandson.first, grandson.second));
              new_transitions.insert(pair(pair(vertex, grandson.first), grandson.second));
            }
            if (grandson.second == 'e' && !epsilon_transitions.contains(pair(vertex, grandson.first))) {
              epsilon_transitions[pair(vertex, grandson.first)] = 'e';
            }
          }
        }
        if (finish_vertexes.contains(son.first)) {
          finish_vertexes[vertex] = 1;
        }
        epsilon_transitions.erase(pair(vertex, son.first));
        graph[vertex].erase(graph[vertex].begin() + i - difference);
        difference += 1;
      }
    }
  }

  void BFS(vector<vector<pair<int, char>>>& graph,
           map<int, int>& finish_vertexes,
           map<pair<int, int>, char>& epsilon_transitions,
           int start, vector<bool>& used1) {


    const int KMagic = 100000000;
    int prev_count_unused = 0;
    int count_unused = KMagic;
    while (!epsilon_transitions.empty() && count_unused != prev_count_unused) {
      queue<int> queue;
      queue.push(start);
      vector<bool> used(used1.size(), false);
      while (!queue.empty()) {
        int v = queue.front();
        used[v] = true;
        create_new_transitions(graph, epsilon_transitions, v, finish_vertexes);
        for (auto son: graph[v]) {
          if (!used[son.first]) {
            queue.push(son.first);
          }
        }
        queue.pop();
      }
      prev_count_unused = count_unused;
      count_unused = 0;
      for (auto v : used) {
        count_unused += 1 - static_cast<int>(v);
      }
      std::cout<<"ВСего осталось " << epsilon_transitions.size() << '\n';
      used1 = used;
    }
    for (int i = 0; i < static_cast<int>(used1.size()); ++i) {
      if (!used1[i]) {
        graph[i].clear();
        if (finish_vertexes.contains(i)) {
          finish_vertexes.erase(i);
        }
      }
    }
  }

  auto AlgorithmTompsona(vector<vector<pair<int, char>>>& graph, int start, vector<char>& sigma, map<int, int>& finish_vertexes) {
    std::set<std::set<int>> finish_vert;
    map<pair<std::set<int>, char>, std::set<int>> new_graph;
    for (int i = 0; i < static_cast<int>(graph.size()); ++i) {
      std::set<int> new_v = {i};
      if (finish_vertexes.contains(i)) {
        finish_vert.insert(new_v);
      }
    }
    for (int i = 0; i < static_cast<int>(graph.size()); ++i) {
      std::set<int> new_v = {i};
      for (auto elem : sigma) {
        bool flag = false;
        std::set<int> new_son;
        for (auto son: graph[i]) {
          if (son.second == elem) {
            new_son.insert(son.first);
            if (finish_vertexes.contains(son.first)) {
              flag = true;
            }
          }
        }
        if (!new_graph.contains(pair(new_v, elem)) && !new_son.empty()) {
          new_graph[pair(new_v, elem)] = new_son;

        }
        if (!finish_vert.contains(new_son) && flag) {
          finish_vert.insert(new_son);
        }
      }
    }
    std::queue<std::set<int>> query;
    std::set<int> st = {start};
    query.push(st);

    std::set<std::set<int>> alreadyWasInSet = {st};
    while (!query.empty()) {
      auto q = query.front();
      query.pop();
      for (auto elem : sigma) {
        std::set<int> new_vertex;
        bool flag = false;
        if (new_graph.contains(pair(q, elem))) {
          for (auto v : new_graph[pair(q, elem)]) {
            std::set<int> v_set = {v};
            if (finish_vert.contains(v_set) && !flag) {
              flag = true;
            }
          }
          for (auto v : new_graph[pair(q, elem)]) {
            new_vertex.insert(v);
          }
        }
        if (!alreadyWasInSet.contains(new_vertex) && !new_vertex.empty()) {
          for (auto symbol : sigma) {
            std::set<int> to_vertex;
            for (auto v : new_vertex) {
              std::set<int> other = {v};
              if (new_graph.contains(pair(other, symbol))) {
                for (auto u: new_graph[pair(other, symbol)]) {
                  to_vertex.insert(u);
                }
              }
            }
            if (!to_vertex.empty()) {
              new_graph[pair(new_vertex, symbol)] = to_vertex;
            }
          }
          alreadyWasInSet.insert(new_vertex);
          query.push(new_vertex);
          if (flag) {
            finish_vert.insert(new_vertex);
          }
        }
      }
    }
    // вернем только те вершины, которые были в очереди и map с переходами и новые финишные сосстояния

    return pair(pair(alreadyWasInSet, finish_vert), new_graph);

  }

  void addTrash(vector<char>& sigma, map<pair<std::set<int>, char>, std::set<int>>& graph, std::set<std::set<int>>& alreadyWasInSet) {
    std::set<int> Trash = {-1};
    bool flag = false;
    for (auto v : alreadyWasInSet) {
      for (auto symbol : sigma) {
        if (symbol != 'e') {
          if (!graph.contains(pair(v, symbol))) {
            flag = true;
            graph[pair(v, symbol)] = Trash;
          }
        }
      }
    }
    if (flag) {
      for (auto elem: sigma) {
        if (elem != 'e') {
          graph[pair(Trash, elem)] = Trash;
        }
      }
      alreadyWasInSet.insert(Trash);
    }
  }

  auto minimization(map<pair<std::set<int>, char>, std::set<int>>& graph, vector<char>& sigma, std::set<std::set<int>>& alreadyWasInSet, std::set<std::set<int>>& finish_vert, std::set<int> start) {
    map<std::set<int>, int> start_map;
    for (auto v : alreadyWasInSet) {
      if (finish_vert.contains(v)) {
        start_map[v] = 1;
      } else {
        start_map[v] = 0;
      }
    }
    int count_class_prev = -10;
    int count_class = 0;
    map<std::set<int>, int> mapOfClass;
    while (count_class_prev != count_class) {
      count_class_prev = count_class;
      count_class = 0;
      map<vector<std::set<int>>, int> mapOfNewVertexes;
      map<std::set<int>, int> mapOfNewClasses;
      map<vector<int>, int> same_transition;
      vector<vector<std::set<int>>> vertexes_by_classes(alreadyWasInSet.size() + 1);
      for (auto v : alreadyWasInSet) {
        vector<int> transitions;
        for (auto symbol : sigma) {
          if (graph.contains(pair(v, symbol))) {
            auto vertex_in_transition = graph[pair(v, symbol)];
            int old_class_equal = start_map[vertex_in_transition];
            transitions.push_back(old_class_equal);
          }
        }
        if (same_transition.contains(transitions)) {
          bool flag = false;
          for (auto v_candidate : vertexes_by_classes[same_transition[transitions]]) {
            if (start_map[v] == start_map[v_candidate] && !flag) {
              mapOfNewClasses[v] = mapOfNewClasses[v_candidate];
              vertexes_by_classes[same_transition[transitions]].push_back(v);
              flag = true;
            }
          }
          if (!flag) {
            vertexes_by_classes[same_transition[transitions]].push_back(v);
            count_class+=1;
            mapOfNewClasses[v] = count_class;
          }
        } else {
          count_class += 1;
          same_transition[transitions] = count_class;
          vertexes_by_classes[same_transition[transitions]].push_back(v);
          mapOfNewClasses[v] = count_class;
        }
      }
      start_map = mapOfNewClasses;
    }

    int colors = count_class;
    std::set<int> finish_v;
    map<pair<int, char>, int> newEdges;
    vector<vector<pair<int, char>>> Mpdka(colors + 1);
    int start_color = 0;
    for (auto v : start_map) {
      auto vertex = v.first;
      int color = v.second;
      if (vertex == start) {
        start_color = color;
      }
      if (finish_vert.contains(vertex)) {
        finish_v.insert(color);
      }
      for (auto symbol : sigma) {
        if (symbol != 'e') {
          auto vertex_to = graph[pair(vertex, symbol)];
          int color_to = start_map[vertex_to];
          if (!newEdges.contains(pair(color, symbol))) {
            newEdges[pair(color, symbol)] = color_to;
            Mpdka[color].push_back(pair(color_to, symbol));
          }
        }
      }
    }
    return pair(Mpdka, pair(finish_v, start_color));
  }

  auto getRegularExpression(vector<vector<pair<int, char>>> Automata, std::set<int>& finish_vertexes, int start) {
    Automata.push_back(vector<pair<int, char>>());
    int sizeOfAutomata = static_cast<int>(Automata.size());
    for (auto finish_v : finish_vertexes) {
      Automata[finish_v].push_back(pair(sizeOfAutomata - 1, 'e'));
    }

    vector<map<int, string>> automataMap(Automata.size());
    vector<map<int, string>> reverseAutomata(Automata.size());


    for (int i = 1; i < sizeOfAutomata; ++i) {
      for (auto v : Automata[i]) {
        if (automataMap[i].contains(v.first)) {
          automataMap[i][v.first] = automataMap[i][v.first] + "+" + (v.second);
          reverseAutomata[v.first][i] = reverseAutomata[v.first][i] + "+" + (v.second);
        } else {
          automataMap[i][v.first] = (v.second);
          reverseAutomata[v.first][i] = (v.second);
        }
      }
    }
    std::set<int> vertexesAll;
    int count = static_cast<int>(automataMap.size());
    for (int i = 1; i < Automata.size() - 1; ++i) {
      vertexesAll.insert(i);
    }

    const int kMagic = 10000000;
    while (count > 3) {
      int idxDelete = start;
      int minDeg = kMagic;
      for (auto i : vertexesAll) {
        int nowDeg;
        if (i != start) {
          nowDeg = static_cast<int>(reverseAutomata[i].size() * Automata[i].size());
          if (nowDeg < minDeg) {
            minDeg = nowDeg;
            idxDelete = i;
          }
        }
      }
      if (idxDelete != start) {
        string klini = "";
        if (automataMap[idxDelete].contains(idxDelete)) {
          klini = "(" + automataMap[idxDelete][idxDelete] + ")" + "*";
          automataMap[idxDelete].erase(idxDelete);
          reverseAutomata[idxDelete].erase(idxDelete);
        }
        for (auto& v_reverse : reverseAutomata[idxDelete]) {
          for (auto& u : automataMap[idxDelete]) {
            if (automataMap[v_reverse.first].contains(u.first)) {
              if (automataMap[v_reverse.first][u.first] != v_reverse.second + klini +  u.second) {
                automataMap[v_reverse.first][u.first] = automataMap[v_reverse.first][u.first] + "+" +
                                                        v_reverse.second + klini + u.second;
                reverseAutomata[u.first][v_reverse.first] = reverseAutomata[u.first][v_reverse.first] + "+" +
                                                            v_reverse.second + klini + u.second;
              }
            } else {
              automataMap[v_reverse.first][u.first] = v_reverse.second + klini + u.second;
              reverseAutomata[u.first][v_reverse.first] = v_reverse.second + klini + u.second;
            }
          }
        }
        for (auto v_rev : reverseAutomata[idxDelete]) {
          if (v_rev.first != idxDelete) {
            automataMap[v_rev.first].erase(idxDelete);
          }
        }

        for (auto u : automataMap[idxDelete]) {
          if (u.first != idxDelete) {
            reverseAutomata[u.first].erase(idxDelete);
          }
        }
        automataMap[idxDelete].clear();
        reverseAutomata[idxDelete].clear();
        vertexesAll.erase(idxDelete);
        count -= 1;
      }
      std::cout << automataMap.size() << '\n';
    }

    string alpha = "e";
    string beta = "e";
    string gamma = "e";
    string delta = "e";
    if (automataMap[start].contains(start)) {
      alpha = automataMap[start][start];
    }
    for (auto v : automataMap[start]) {
      if (v.first != start) {
        beta = v.second;
        if (automataMap[v.first].contains(v.first)) {
          gamma = automataMap[v.first][v.first];
        }
        if (automataMap[v.first].contains(start)) {
          delta = automataMap[v.first][start];
        }
      }
    }
    string ans = "(" + alpha + ")*" + "("+beta+")" + "("+ gamma + "+" + delta + "("+ alpha + ")"+ "*" + beta + ")" + "*";
    return ans;
  }
 public:
  DFA(NFA& automata) {
    auto graph = automata.getGraph();
    count = automata.getCount();
    auto finish_vertexes = automata.getFinish();


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

  bool check(string& expression) {
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
//
//int main() {
//
//  string s = "(a+b)*$a+e";
//  NFA automata(s);
//  DFA aut(automata);
//  aut.makeMinimization();
//  aut.getDFA();
//
//  DFA aut2(s);
//  string s2 = "aaaabbb";
//  std::cout << aut2.getRegular();
//
//
//}

