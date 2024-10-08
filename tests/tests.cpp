#include <gtest/gtest.h>
#include "src/DFA.cpp"
#include <string>
#include <fstream>

using std::string;

TEST (GetRegExp, GetDFARegExp) {
  string s = "(a$b)+(e+a)*";
  DFA a(s);
  DFA b(a.getRegular());
  EXPECT_TRUE(b.check("ab"));
  EXPECT_TRUE(b.check(""));
  EXPECT_TRUE(b.check("aaaaaaaa"));
  EXPECT_FALSE(b.check("aba"));
  EXPECT_FALSE(b.check("abaaaaaa"));
  EXPECT_FALSE(b.check("abab"));
  EXPECT_FALSE(b.check("abb"));
}

TEST (GetRegExp, GetNFARegExp) {
  string ss = "(b+(a$a$(b$a)*$a))^";
  NFA a(ss);
  std::cout << a.getRegular();
  NFA b(a.getRegular());
  EXPECT_TRUE(b.check("b"));
  EXPECT_TRUE(b.check("bbbbb"));
  EXPECT_TRUE(b.check("baaaaababaa"));
  EXPECT_TRUE(b.check("aabaabaaa"));
  EXPECT_FALSE(b.check(""));
  EXPECT_FALSE(b.check("bab"));
  EXPECT_FALSE(b.check("aab"));
  EXPECT_FALSE(b.check("baabaaa"));
}
TEST (CreateByRegExp, CreateNFAByRegExp) {
  NFA a("a$b+(e+a)*");
  EXPECT_TRUE(a.check("ab"));
  EXPECT_TRUE(a.check(""));
  EXPECT_TRUE(a.check("aaaaaaaa"));
  EXPECT_FALSE(a.check("aba"));
  EXPECT_FALSE(a.check("abaaaaaa"));
  EXPECT_FALSE(a.check("abab"));
  EXPECT_FALSE(a.check("abb"));
}

TEST (CreateByRegExp, CreateDFAByRegExp) {
  DFA a("(b+a$a$(b$a)*$a)^");
  EXPECT_TRUE(a.check("b"));
  EXPECT_TRUE(a.check("bbbbb"));
  EXPECT_TRUE(a.check("baaaaababaa"));
  EXPECT_TRUE(a.check("aabaabaaa"));
  EXPECT_FALSE(a.check(""));
  EXPECT_FALSE(a.check("bab"));
  EXPECT_FALSE(a.check("aab"));
  EXPECT_FALSE(a.check("baabaaa"));
}


TEST (NFA, CreateNFAByString1) {
  std::streambuf* cinbuf = std::cin.rdbuf();
  std::ifstream in("../tests/text_create_fromString.txt");
  std::cin.rdbuf(in.rdbuf());
  NFA a = createNFAfromString();
  EXPECT_TRUE(a.check("abba"));
  EXPECT_TRUE(a.check("abbbbbbbb"));
  EXPECT_TRUE(a.check("abbbbbaaaaaaaaaa"));
  EXPECT_TRUE(a.check(""));
  EXPECT_FALSE(a.check("baaaa"));
  EXPECT_FALSE(a.check("abab"));
  EXPECT_FALSE(a.check("abcab"));
  std::cin.rdbuf(cinbuf);
}

TEST (NFA, CreateNFAByString2) {
  std::streambuf* cinbuf = std::cin.rdbuf();
  std::ifstream in("../tests/text_create_fromString2.txt");
  std::cin.rdbuf(in.rdbuf());
  NFA a = createNFAfromString();
  EXPECT_TRUE(a.check(""));
  EXPECT_TRUE(a.check("ab"));
  EXPECT_TRUE(a.check("abaababb"));
  EXPECT_TRUE(a.check("abababa"));
  EXPECT_TRUE(a.check("abababb"));
  EXPECT_FALSE(a.check("b"));
  EXPECT_FALSE(a.check("bbba"));
  EXPECT_FALSE(a.check("abaaab"));
  EXPECT_FALSE(a.check("abaabbb"));
  std::cin.rdbuf(cinbuf);
}

TEST (MDFA, CreateMDFA1) {
  DFA b("b+(b$(a$a)*$(a+b)*$a$a$b*)$(1+b$a$b*)");
  b.makeMinimization();
  EXPECT_EQ(b.getCount(), 9);
}

TEST (MDFA, CreateMDFA2) {
  DFA b("(b+a$a$(b$a)*$a)^");
  b.makeMinimization();
  EXPECT_EQ(b.getCount(), 5);
}

TEST (MDFA, CreateMDFA3) {
  DFA b("(((1+1)*)^+(1*$1^)*)*");
  b.makeMinimization();
  EXPECT_EQ(b.getCount(), 2);
}


TEST (DFA, CreateDFAByNFA) {
  std::streambuf* cinbuf = std::cin.rdbuf();
  std::ifstream in("../tests/text_create_fromString.txt");
  std::cin.rdbuf(in.rdbuf());
  NFA a = createNFAfromString();
  DFA b(a);
  EXPECT_TRUE(b.check("abba"));
  EXPECT_TRUE(b.check("abbbbbbbb"));
  EXPECT_TRUE(b.check("abbbbbaaaaaaaaaa"));
  EXPECT_TRUE(b.check(""));
  EXPECT_FALSE(b.check("baaaa"));
  EXPECT_FALSE(b.check("abab"));
  EXPECT_FALSE(b.check("abcab"));
  std::cin.rdbuf(cinbuf);
}
