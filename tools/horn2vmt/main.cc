// Supported by National Laboratories, a multi-mission laboratory managed and
// operated by National Technology and En- gineering Solutions of Sandia, LLC,
// a wholly owned subsidiary of Honeywell In- ternational, Inc., for the U.S.
// Department of Energy’s National Nuclear Security Administration under
// contract DE-NA0003525.


#include <fmt/ostream.h>
#include <fmt/format.h>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/range.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/combine.hpp>
#include <boost/make_unique.hpp>
#include <fstream>
#include <gmpxx.h>
#include <llvm/ADT/DepthFirstIterator.h>
#include <llvm/ADT/GraphTraits.h>
#include <llvm/ADT/PostOrderIterator.h>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <z3++.h>

using namespace std;

// These appear to be needed because the template system can't figure out
// they're all subclasses af z3::ast?  But also using operator<< on them
// doesn't work; has to be the formatter direct overload.
template <>
struct fmt::formatter<z3::expr> {
  auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && *it != '}') {
      std::cerr << "invalid format\n";
      exit(1);
    }
    return it;
  }

  template <typename FormatContext>
  auto format(const z3::expr& e, FormatContext& ctx) -> decltype(ctx.out()) {
    return format_to(
        ctx.out(),
        "{}",
        e.to_string());
  }
};

template <>
struct fmt::formatter<z3::func_decl> {
  auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && *it != '}') {
      std::cerr << "invalid format\n";
      exit(1);
    }
    return it;
  }

  template <typename FormatContext>
  auto format(const z3::func_decl& f, FormatContext& ctx) -> decltype(ctx.out()) {
    return format_to(
        ctx.out(),
        "{}",
        f.to_string());
  }
};

template <>
struct fmt::formatter<z3::sort> {
  auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && *it != '}') {
      std::cerr << "invalid format\n";
      exit(1);
    }
    return it;
  }

  template <typename FormatContext>
  auto format(const z3::sort& f, FormatContext& ctx) -> decltype(ctx.out()) {
    return format_to(
        ctx.out(),
        "{}",
        f.to_string());
  }
};

//^----------------------------------------------------------------------------^
// Section 1. This section basically has a ton of boilerplate and support
// functions that make the horn2vmt more intuitive to write. Things like
// iterating over, visiting, and rewriting expressions.

namespace {
#define FATAL(...) do { \
  fmt::print(std::cerr, "FATAL at {}:{}: {}\n", __FILE__, __LINE__, \
             __VA_ARGS__); \
  abort(); \
} while (false);

template<class T>
class ProxyHolder {
 public:
  ProxyHolder(const T& t) : t_(t) {}
  T* operator->() { return std::addressof(t_); }
 private:
  T t_;
};

//! Iterator over immediate args of an expression
class ExprArgIterator {
 public:
  using value_type = z3::expr;
  using reference = z3::expr;
  using pointer = z3::expr*;
  using iterator_category = std::input_iterator_tag;
  using difference_type = int;
  
  ExprArgIterator() : i_(0) {}
  ExprArgIterator(const z3::expr& e, unsigned i) : e_(e), i_(i) {}

  // static constructor
  inline static ExprArgIterator begin(const z3::expr& e) {
    return ExprArgIterator(e, 0);
  }

  // static constructor
  inline static ExprArgIterator end(const z3::expr& e) {
    switch (e.kind()) {
      case Z3_QUANTIFIER_AST:
        return ExprArgIterator(e, 1);
      case Z3_APP_AST:
        return ExprArgIterator(e, e.num_args());
      default:
        return ExprArgIterator(e, 0);
    }
  }

  inline bool operator==(const ExprArgIterator& r) const {
    return z3::eq(*e_, *r.e_) && i_ == r.i_;
  }

  inline bool operator!=(const ExprArgIterator& it) const {
    return !(*this == it);
  }

  inline z3::expr operator*() const {
    if (e_->is_quantifier() && i_ == 0) {
      return e_->body();
    } else {
      return e_->arg(i_);
    }
  }
  inline z3::expr operator*() {
    if (e_->is_quantifier() && i_ == 0) {
      return e_->body();
    } else {
      return e_->arg(i_);
    }
  }
  inline ExprArgIterator& operator++() { ++i_; return *this; }
  
  inline ExprArgIterator operator++(int) {
    ExprArgIterator tmp = *this; ++*this; return tmp;
  }

  ProxyHolder<value_type> operator->() const {
    return ProxyHolder<value_type>(**this);
  }

 private:
  boost::optional<z3::expr> e_;
  unsigned i_;
};
}

namespace z3 {
//! Wraps a z3 expr so it can be used inside various collections. Equality,
//! hashing, and ordering have to be done properly.
class ExprWrapper {
 public:
  ExprWrapper(const z3::expr& e) : e_(e) {}
  // convert implicitly back to z3::expr

  inline operator z3::expr() const { return e_; }

  inline bool operator==(const ExprWrapper& rhs) const {
    return z3::eq(e_, rhs.e_);
  }

  inline bool operator!=(const ExprWrapper& rhs) const {
    return !(*this == rhs);
  }

  inline std::size_t hash() const { return e_.hash(); }
  // arbitrary stable ordering
  inline bool operator<(const ExprWrapper& rhs) const {
    return hash() < rhs.hash();
  }

 private:
  z3::expr e_;
};

struct HashExpr {
  inline std::size_t operator()(const ExprWrapper& a) const { return a.hash(); }
  inline std::size_t operator()(const z3::ast& a) const { return a.hash(); }
};

struct EqualToExpr {
  inline bool operator()(const ExprWrapper& a, const ExprWrapper& b) const {
    return a == b;
  }
  inline bool operator()(const ast& a, const ast& b) const {
    return eq(a, b);
  }
};

struct CompareExpr {
  bool operator()(const ast& a, const ast& b) const {
    return a.hash() < b.hash();
  }
};

//^----------------------------------------------------------------------------^

//! Iterator over z3::expr_vector
class ExprVectorIterator {
 public:
  using self_type = ExprVectorIterator;
  using value_type = z3::expr;
  using reference = value_type;
  using pointer = value_type*;
  using iterator_category = std::input_iterator_tag;
  using difference_type = int;

  ExprVectorIterator(int i, const z3::expr_vector& v) : index_(i), vec_(v) {}
  
  inline self_type& operator++() {
    index_++;
    return *this;
  }
  
  self_type operator++(int) {
    self_type i = *this;
    index_++;
    return i;
  }

  inline self_type& operator--() { // preincrement
    index_--;
    return *this;
  }

  self_type operator--(int) { // postincrement
    self_type i = *this;
    index_--;
    return i;
  }
  
  inline value_type operator*() const {
    return vec_[index_];
  }

  inline ProxyHolder<value_type> operator->() const {
    return ProxyHolder<decltype(**this)>(vec_[index_]);
  }
  
  inline bool operator==(const self_type& rhs) const {
    return index_ == rhs.index_ && &vec_ == &rhs.vec_;
  }

  inline bool operator!=(const self_type& rhs) const { return !(*this == rhs); }

 private:
  int index_;
  const z3::expr_vector& vec_;
};

ExprVectorIterator begin(const z3::expr_vector& v) { return ExprVectorIterator(0, v); }
ExprVectorIterator end(const z3::expr_vector& v) { return ExprVectorIterator(v.size(), v); }
} // end namespace z3

namespace llvm {
//! Graph traits instance for z3::expr. Gives us LLVM the ability to use LLVM
//! graph traversals
template <>
class GraphTraits<z3::expr> {
 public:
  using NodeRef = z3::expr;
  using ChildIteratorType = ExprArgIterator;

  static NodeRef getEntryNode(const z3::expr& e) { return e; }

  static ChildIteratorType child_begin(NodeRef n) {
    return ExprArgIterator::begin(n);
  }
  static ChildIteratorType child_end(NodeRef n) {
    return ExprArgIterator::end(n);
  }
};
} // end namespace llvm


namespace {
class Logger {
  int level_;
  uint64_t count_;
  std::ofstream mirror_file_;

 public:
  Logger() : level_(0), count_(0) {}

  void set_level(int level) { level_ = level; }

  bool ShouldLog(int level) { return level <= level_; }

  void set_mirror_file(std::string filename) {
    mirror_file_ = std::ofstream(filename);
  }

  template <typename... Args> void Log(int level, const char *fmt,
                                       const Args&... args) {
    if (ShouldLog(level)) {
      auto fmt_msg = [&](std::ostream& os) {
        fmt::print(os, fmt, std::forward<const Args>(args)...);
        fmt::print(os, "\n");
      };
      //fmt::print(std::cerr, "[{:06d}] ", count_);
      fmt_msg(std::cerr);
      if (mirror_file_.is_open())
        fmt_msg(mirror_file_);
      ++count_;
    }
  }
  
  template <typename... Args> void LogOpenFold(int level, const char *fmt,
                                               const Args&... args) {
    if (ShouldLog(level)) {
      //fmt::print(std::cerr, "[{:06d}] ", count_);
      auto fmt_msg = [&](std::ostream& os) {
        fmt::print(os, fmt, std::forward<const Args>(args)...);
        fmt::print(os, "{{{{{{");
        fmt::print(os, "\n");
      };
      fmt_msg(std::cerr);
      if (mirror_file_.is_open())
        fmt_msg(mirror_file_);
      ++count_;
    }
  }
  
  template <typename... Args> void LogCloseFold(int level, const char *fmt,
                                                const Args&... args) {
    if (ShouldLog(level)) {
      //fmt::print(std::cerr, "[{:06d}] ", count_);
      auto fmt_msg = [&](std::ostream& os) {
        fmt::print(os, fmt, std::forward<const Args>(args)...);
        fmt::print(os, "}}}}}}");
        fmt::print(os, "\n");
      };
      fmt_msg(std::cerr);
      if (mirror_file_.is_open())
        fmt_msg(mirror_file_);
      ++count_;
    }
  }

  void Log(int level, const char *msg) {
    if (ShouldLog(level)) {
      //fmt::print(std::cerr, "[{:06d}] ", count_);
      auto fmt_msg = [&](std::ostream& os) {
        fmt::print(os, msg);
        fmt::print(os, "\n");
      };
      fmt_msg(std::cerr);
      if (mirror_file_.is_open())
        fmt_msg(mirror_file_);
      ++count_;
    }
  }
};

// This type is here because I first tried to use llvm::Optional<StateVarRef>
// but then clients needed to explicitly call get() to remove the reference
// wrapper and that's dumb so here we are.
template <typename T>
class OptionalRef {
  using Ref = std::reference_wrapper<T>;
 public:
  OptionalRef(const T& v) : v_(v) {}
  OptionalRef() {}

  inline operator bool() const {
    return bool(v_);
  }

  inline operator boost::optional<Ref>() const {
    return v_;
  }

  inline const T& operator*() const {
    return v_->get();
  }

  inline const T *operator->() const {
    return &v_->get();
  }

  inline OptionalRef& operator=(const T& v) {
    v_ = v;
    return *this;
  }

 private:
  boost::optional<Ref> v_;
};

//! Set of expressions. I'm careful to use z3::ExprWrapper here because if you
//! use == or != on sets, it will call x == y (for elements x and y from those
//! sets). Because of the EqualityComparable
//! requirement in the c++ docs, I guess? I don't know. In any case, if you
//! have a set<z3::expr>, these == calls don't have the right semantics.
//!
//! The main annoyance is you may not always be able to do
//!
//! for (auto& e : <set>) { e.expr_method() }
//!
//! instead you need to do:
//!
//! for (const z3::expr& e : <set>) { e.expr_method(); }
using ExprSet = std::unordered_set<z3::ExprWrapper, z3::HashExpr, z3::EqualToExpr>;


//! Be wary when comparing with equality on maps (see comment on ExprSet
//! above).
template <typename T>
using ExprMap = std::unordered_map<z3::expr, T, z3::HashExpr, z3::EqualToExpr>;
template <typename T>
using ExprMultiMap = std::unordered_multimap<z3::ExprWrapper, T, z3::HashExpr, z3::EqualToExpr>;

template <typename T>
using AstMap = std::unordered_map<z3::ast, T, z3::HashExpr, z3::EqualToExpr>;
template <typename T>
using SortMap = std::unordered_map<z3::sort, T, z3::HashExpr, z3::EqualToExpr>;

static boost::iterator_range<ExprArgIterator> ExprArgs(const z3::expr& e) {
  return boost::make_iterator_range(
      ExprArgIterator::begin(e), ExprArgIterator::end(e));
}

//^----------------------------------------------------------------------------^

//! Visit expressions by kind
template <typename SubClass, typename RetTy=void, typename ExprTy=z3::expr>
class ExprVisitor {
 public:
  // override this in a subclass if you are visiting some type that you can get
  // a node from
  z3::expr get_visit_node(const ExprTy& e) {
    return e;
  }

  // Call this to start things off
  RetTy visit(const ExprTy& e) {
    auto node = static_cast<SubClass*>(this)->get_visit_node(e);
    switch (node.kind()) {
      case Z3_VAR_AST:
        return static_cast<SubClass*>(this)->visitVAR(e);
      case Z3_QUANTIFIER_AST:
        return static_cast<SubClass*>(this)->visitQUANTIFIER(e);
      case Z3_APP_AST:
      case Z3_NUMERAL_AST:
        switch (node.decl().decl_kind()) {
#define HANDLE_KIND(OP) case Z3_OP_##OP: return static_cast<SubClass*>(this)->visit##OP(e);
          HANDLE_KIND(UNINTERPRETED);
          // logical
          HANDLE_KIND(TRUE);
          HANDLE_KIND(FALSE);
          HANDLE_KIND(EQ);
          HANDLE_KIND(IFF);
          HANDLE_KIND(DISTINCT);
          HANDLE_KIND(NOT);
          HANDLE_KIND(AND);
          HANDLE_KIND(OR);
          HANDLE_KIND(XOR);
          HANDLE_KIND(ITE);
          HANDLE_KIND(IMPLIES);

          // int
          HANDLE_KIND(ANUM);
          HANDLE_KIND(ADD);
          HANDLE_KIND(SUB);
          HANDLE_KIND(UMINUS);
          HANDLE_KIND(MUL);
          HANDLE_KIND(DIV);
          HANDLE_KIND(IDIV);
          HANDLE_KIND(REM);
          HANDLE_KIND(MOD);
          HANDLE_KIND(LE);
          HANDLE_KIND(LT);
          HANDLE_KIND(GE);
          HANDLE_KIND(GT);

          HANDLE_KIND(TO_REAL);
          HANDLE_KIND(TO_INT);
          HANDLE_KIND(IS_INT);
          HANDLE_KIND(POWER);

          // array
          HANDLE_KIND(STORE);
          HANDLE_KIND(SELECT);
          HANDLE_KIND(CONST_ARRAY);
          HANDLE_KIND(AS_ARRAY);
          HANDLE_KIND(ARRAY_DEFAULT);
          HANDLE_KIND(ARRAY_MAP);
          HANDLE_KIND(ARRAY_EXT);

          HANDLE_KIND(SET_UNION);
          HANDLE_KIND(SET_INTERSECT);
          HANDLE_KIND(SET_DIFFERENCE);
          HANDLE_KIND(SET_COMPLEMENT);
          HANDLE_KIND(SET_SUBSET);

          // bitvec
          HANDLE_KIND(BNUM);
          HANDLE_KIND(BIT1);
          HANDLE_KIND(BIT0);
          HANDLE_KIND(BADD);
          HANDLE_KIND(BMUL);
          HANDLE_KIND(BSUB);
          HANDLE_KIND(BSDIV);
          HANDLE_KIND(BSDIV0);
          HANDLE_KIND(BSDIV_I);
          HANDLE_KIND(BUDIV);
          HANDLE_KIND(BUDIV0);
          HANDLE_KIND(BUDIV_I);
          HANDLE_KIND(BSREM);
          HANDLE_KIND(BSREM0);
          HANDLE_KIND(BSREM_I);
          HANDLE_KIND(BUREM);
          HANDLE_KIND(BUREM0);
          HANDLE_KIND(BUREM_I);
          HANDLE_KIND(BSMOD);
          HANDLE_KIND(BSMOD_I);
          HANDLE_KIND(BSHL);
          HANDLE_KIND(BASHR);
          HANDLE_KIND(BLSHR);
          HANDLE_KIND(CONCAT);
          HANDLE_KIND(EXTRACT);
          HANDLE_KIND(SIGN_EXT);
          HANDLE_KIND(ZERO_EXT);
          HANDLE_KIND(SLT);
          HANDLE_KIND(SLEQ);
          HANDLE_KIND(SGT);
          HANDLE_KIND(SGEQ);
          HANDLE_KIND(ULT);
          HANDLE_KIND(ULEQ);
          HANDLE_KIND(UGT);
          HANDLE_KIND(UGEQ);
          HANDLE_KIND(BNEG);
          HANDLE_KIND(BAND);
          HANDLE_KIND(BOR);
          HANDLE_KIND(BXOR);
          HANDLE_KIND(BNOR);
          HANDLE_KIND(BNOT);
          HANDLE_KIND(REPEAT);
          HANDLE_KIND(BREDAND);
          HANDLE_KIND(BCOMP);
          HANDLE_KIND(ROTATE_LEFT);
          HANDLE_KIND(ROTATE_RIGHT);
          HANDLE_KIND(EXT_ROTATE_LEFT);
          HANDLE_KIND(EXT_ROTATE_RIGHT);
          HANDLE_KIND(INT2BV);
          HANDLE_KIND(BV2INT);
          HANDLE_KIND(CARRY);
          HANDLE_KIND(XOR3);
          HANDLE_KIND(BSMUL_NO_OVFL);
          HANDLE_KIND(BUMUL_NO_OVFL);
          HANDLE_KIND(BSMUL_NO_UDFL);

          HANDLE_KIND(AGNUM);

          default:
          FATAL("unhandled Z3 operator");
#undef HANDLE_KIND

        }

      default:
        FATAL("unhandled AST");
    }
  }
    
  // default case. you must override this if return type is not void
  void visitExpr(const z3::expr&) {} // ignore
    
    
  RetTy visitVAR(const ExprTy& e) {
    return static_cast<SubClass*>(this)->visitExpr(e);
  }

  RetTy visitQUANTIFIER(const ExprTy& e) {
    return static_cast<SubClass*>(this)->visitExpr(e);
  }

#define DELEGATE(KIND) RetTy visit##KIND(const ExprTy& e) { \
  return static_cast<SubClass*>(this)->visitExpr(e); }
  DELEGATE(UNINTERPRETED);
  DELEGATE(TRUE);
  DELEGATE(FALSE);
  DELEGATE(EQ);
  DELEGATE(IFF);
  DELEGATE(DISTINCT);
  DELEGATE(NOT);
  DELEGATE(AND);
  DELEGATE(OR);
  DELEGATE(XOR);
  DELEGATE(ITE);
  DELEGATE(IMPLIES);

  DELEGATE(ANUM);
  DELEGATE(ADD);
  DELEGATE(SUB);
  DELEGATE(UMINUS);
  DELEGATE(MUL);
  DELEGATE(DIV);
  DELEGATE(IDIV);
  DELEGATE(REM);
  DELEGATE(MOD);
  DELEGATE(LE);
  DELEGATE(LT);
  DELEGATE(GE);
  DELEGATE(GT);

  DELEGATE(TO_REAL);
  DELEGATE(TO_INT);
  DELEGATE(IS_INT);
  DELEGATE(POWER);

  DELEGATE(BNUM);
  DELEGATE(BIT1);
  DELEGATE(BIT0);
  DELEGATE(BADD);
  DELEGATE(BSUB);
  DELEGATE(BMUL);
  DELEGATE(BSDIV);
  DELEGATE(BSDIV0);
  DELEGATE(BSDIV_I);
  DELEGATE(BUDIV);
  DELEGATE(BUDIV0);
  DELEGATE(BUDIV_I);
  DELEGATE(BSREM);
  DELEGATE(BSREM0);
  DELEGATE(BSREM_I);
  DELEGATE(BUREM);
  DELEGATE(BUREM0);
  DELEGATE(BUREM_I);
  DELEGATE(BSMOD);
  DELEGATE(BSMOD_I);
  DELEGATE(BSHL);
  DELEGATE(BASHR);
  DELEGATE(BLSHR);
  DELEGATE(CONCAT);
  DELEGATE(EXTRACT);
  DELEGATE(SIGN_EXT);
  DELEGATE(ZERO_EXT);
  DELEGATE(SLT);
  DELEGATE(SLEQ);
  DELEGATE(SGT);
  DELEGATE(SGEQ);
  DELEGATE(ULT);
  DELEGATE(ULEQ);
  DELEGATE(UGT);
  DELEGATE(UGEQ);
  DELEGATE(BNEG);
  DELEGATE(BAND);
  DELEGATE(BOR);
  DELEGATE(BXOR);
  DELEGATE(BNOR);
  DELEGATE(BNOT);
  DELEGATE(REPEAT);
  DELEGATE(BREDAND);
  DELEGATE(BCOMP);
  DELEGATE(ROTATE_LEFT);
  DELEGATE(ROTATE_RIGHT);
  DELEGATE(EXT_ROTATE_LEFT);
  DELEGATE(EXT_ROTATE_RIGHT);
  DELEGATE(INT2BV);
  DELEGATE(BV2INT);
  DELEGATE(CARRY);
  DELEGATE(XOR3);
  DELEGATE(BSMUL_NO_OVFL);
  DELEGATE(BUMUL_NO_OVFL);
  DELEGATE(BSMUL_NO_UDFL);

  DELEGATE(STORE);
  DELEGATE(SELECT);
  DELEGATE(CONST_ARRAY);
  DELEGATE(AS_ARRAY);
  DELEGATE(ARRAY_DEFAULT);
  DELEGATE(ARRAY_MAP);
  DELEGATE(ARRAY_EXT);

  DELEGATE(AGNUM);
      
  DELEGATE(SET_UNION);
  DELEGATE(SET_INTERSECT);
  DELEGATE(SET_DIFFERENCE);
  DELEGATE(SET_COMPLEMENT);
  DELEGATE(SET_SUBSET);

#undef DELEGATE
 
 protected:
  ExprVisitor() = default;
  ~ExprVisitor() = default;
 
};

//!
//! Rewriter for DAGs. Calls SubClass::visit to rewrite each node. Nodes that
//! have been rewritten can be accessed with lookup().
//!
//! \param SubClass - cocrete subclass, crtp style
//!
//! \param GraphT - there is a graph traits for this
//!
//! \param Ret - rewriter returns this type
//!
//! \param Set - for stored visited nodes
//!
//! \param Cache - for caching intermediate rewrites
template <typename SubClass, typename GraphT,
          typename Ret=typename llvm::GraphTraits<GraphT>::NodeRef,
          typename Set=std::unordered_set<
              typename llvm::GraphTraits<GraphT>::NodeRef>,
          typename Cache=std::unordered_map<
              typename llvm::GraphTraits<GraphT>::NodeRef, Ret>>
class Rewriter {
 public:
  using NodeRef = typename llvm::GraphTraits<GraphT>::NodeRef;
  
  using RewriteIterator = llvm::po_ext_iterator<NodeRef, 
        Rewriter<SubClass, NodeRef, Ret, Set, Cache>>;
  
  inline int64_t num_visits() const { return num_visits_; }
  inline void set_num_visits(unsigned n = 0) { num_visits_ = n; }

  Ret Rewrite(NodeRef n) {
    for (auto it = static_cast<SubClass*>(this)->rewrite_begin(n),
         ie = static_cast<SubClass*>(this)->rewrite_end(n); it != ie; ++it) {
      ++num_visits_;
      auto ret = static_cast<SubClass*>(this)->visit(*it);
      cache_.insert({static_cast<SubClass*>(this)->get_rewrite_node(*it), ret});
    }
    return lookup(n);
  }

  //! Can be defined in a subclass if the iterator type is custom
  inline NodeRef get_rewrite_node(NodeRef n) const {
    return n;
  }

  inline const Ret& lookup(NodeRef n) const {
    return cache_.at(n);
  }
  inline const Ret& Lookup(NodeRef n) const {
    return lookup(n);
  }
  inline bool is_in_cache(NodeRef e) const {
    return cache_.find(e) != cache_.end();
  }
  
  inline void Reset() {
    visited_.clear();
    cache_.clear();
    num_visits_ = 0;
  }
  
  //! iterate in rewrite order over e
  inline RewriteIterator rewrite_begin(NodeRef e) {
    return llvm::po_ext_begin(e, *this);
  }

  inline RewriteIterator rewrite_end(NodeRef e) {
    return llvm::po_ext_end(e, *this);
  }
  
  inline void FinishPostorder(NodeRef) {}

  inline bool VisitPreorder(const llvm::Optional<NodeRef>& from, NodeRef to) {
    bool b = false;
    if (cache_.find(to) == cache_.end()) {
      b = llvm::po_iterator_storage<Set, true>(visited_).insertEdge(from,
                                                                      to);
    }
    return b;
  }

 protected:
  Set visited_;
  Cache cache_;
  int64_t num_visits_;
  
  template <typename... Args>
  Rewriter(Args&&... args) 
      : visited_(std::forward<Args>(args)...), num_visits_(0) {}
  Rewriter(const Rewriter&) = delete;
  ~Rewriter() = default;
};


//! This class allows one to use a custom visited set. This can make many
//! different traversals relatively straightforward to implement. Or at least
//! modular.
template <typename SubClass,
         // Rewriter returns this type
         typename Ret=z3::expr,
         // What holds visited nodes
         typename Set=ExprSet>
class ExprRewriter : public Rewriter<SubClass, z3::expr, Ret, Set,
    ExprMap<Ret>> {
 public:
  using RewriterT = Rewriter<SubClass, z3::expr, Ret, Set,
        ExprMap<Ret>>;

  // using RewriterT::Rewrite;
  // using RewriterT::Lookup;
  using RewriterT::lookup;
  // using RewriterT::VisitAndCache;
  // using RewriterT::num_visits;
  // using RewriterT::set_num_visits;
  // using RewriterT::rewrite_begin;
  // using RewriterT::rewrite_end;

 protected:
  using RewriterT::cache_;

  //! constructor passes args to the custom set type
  template <typename... Args>
  ExprRewriter(Args&&... args) 
      : RewriterT(std::forward<Args>(args)...) {}
  ExprRewriter(const ExprRewriter&) = delete;
  ExprRewriter& operator=(const ExprRewriter&) = delete;
  ~ExprRewriter() = default;
  
  // XXX rename to arg
  inline Ret Arg(const z3::expr& e, unsigned i) const {
    return lookup(e.arg(i));
  }
  inline Ret arg(const z3::expr& e, unsigned i) const {
    return Arg(e, i);
  }
  
  // XXX rename to new_args_expr_vector
  inline z3::expr_vector NewArgsExprVector(const z3::expr& e) const {
    z3::expr_vector args(e.ctx());
    for (unsigned i = 0; i < e.num_args(); i++) {
      args.push_back(Arg(e,i));
    }
    return args;
  }
  
  // XXX rename to new_args and better yet delete
  inline std::vector<Ret> NewArgs(const z3::expr& e) const {
    std::vector<Ret> args;
    args.reserve(e.num_args());
    for (unsigned i = 0; i < e.num_args(); i++) {
      args.push_back(Arg(e,i));
    }
    return args;
  }

  //! Iterates over args in the cache
  template <typename Cache>
  class CachedArgIterator {
   public:
    using difference_type = std::ptrdiff_t;
    using value_type = z3::expr;
    using reference = z3::expr&;
    using pointer = z3::expr*;
    using iterator_category = std::forward_iterator_tag;

    CachedArgIterator(const Cache& c, ExprArgIterator it) 
        : cache_(c), it_(it) {}

    bool operator==(const CachedArgIterator& other) {
      return &cache_ == &other.cache_ && it_ == other.it_;
    }

    bool operator!=(const CachedArgIterator& other) {
      return !(*this == other);
    }

    const z3::expr& operator*() const {
      return cache_.at(*it_);
    }

    CachedArgIterator& operator++() { // preincrement
      ++it_;
      return *this;
    }
    
    CachedArgIterator operator++(int) { // postincrement
      CachedArgIterator tmp = *this;
      ++*this;
      return tmp;
    }
    
   private:
    const Cache& cache_;
    ExprArgIterator it_;
  };

  using ArgIterator = CachedArgIterator<ExprMap<Ret>>;

  //! iterate over rewritten args of e, in order
  ArgIterator args_begin(const z3::expr& e) const {
    return ArgIterator(cache_, ExprArgIterator::begin(e));
  }
  
  ArgIterator args_end(const z3::expr& e) const {
    return ArgIterator(cache_, ExprArgIterator::end(e));
  }
};


//! Manages a substitution over z3::exprs
class ExprSubstitution {
  z3::expr_vector src_, dst_;
 public:

  explicit ExprSubstitution(z3::context& ctx) : src_(ctx), dst_(ctx) {}
  ExprSubstitution(const ExprSubstitution& other);

  z3::context& ctx();

  inline const z3::expr_vector& src() const { return src_; }

  unsigned Size() const { return src_.size(); }

  void Clear();
  
  // substitute eold for enew
  void AddSubstitution(const z3::expr& eold, const z3::expr& enew);

  template <typename T>
  void AddSubstitution(const T& src, const T& dst) {
    for (auto it_src = begin(src), ie_src = end(src),
         it_dst = begin(dst), ie_dst = end(dst);
         it_src != ie_src && it_dst != ie_dst;
         ++it_src, ++it_dst) {
      AddSubstitution(*it_src, *it_dst);
    }
  }
  
  // perform substitution on e
  z3::expr operator()(const z3::expr& e) const;

  // parallel composition
  ExprSubstitution operator&(const ExprSubstitution& other) const;

  // overrides
  
  void Print(std::ostream& out) const;
  
};

ExprSubstitution::ExprSubstitution(const ExprSubstitution& other) 
    : src_(other.src_.ctx()), dst_(other.dst_.ctx()) {
  // z3::expr_vector's copy constructor SHARES POINTERS WITH THE SOURCE
  // INSTANCE.  so we need to manually copy the entries from other into our
  // src_ and dst_
  AddSubstitution(other.src_, other.dst_);
}

z3::context& ExprSubstitution::ctx() {
  return src_.ctx();
}

void ExprSubstitution::Clear() {
  src_ = z3::expr_vector(ctx());
  dst_ = z3::expr_vector(ctx());
}

void ExprSubstitution::AddSubstitution(const z3::expr& eold,
                                       const z3::expr& enew) {
  assert(z3::eq(eold.get_sort(), enew.get_sort()));
  src_.push_back(eold); 
  dst_.push_back(enew);
}

z3::expr ExprSubstitution::operator()(const z3::expr& e) const {
  auto ret = e;
  return ret.substitute(src_, dst_);
  //return rewrite(e);
}

ExprSubstitution
ExprSubstitution::operator&(const ExprSubstitution& other) const {
  ExprSubstitution ret(*this);
  ret.AddSubstitution(other.src_, other.dst_);
  return ret;
}

void ExprSubstitution::Print(std::ostream& out) const {
  fmt::print("ExprSubstitution:\n");
  assert(src_.size() == dst_.size());
  for (unsigned i = 0; i < src_.size(); i++) {
    auto src_elt = src_[i];
    auto dst_elt = dst_[i];
    fmt::print(out, "    {} -> {}\n", src_elt, dst_elt);
  }
}

//^----------------------------------------------------------------------------^

//! A rewriter that does substitution, therefore subexpression substitutions
//! are cached
class CachingExprSubstitution : public ExprRewriter<CachingExprSubstitution>,
    public ExprVisitor<CachingExprSubstitution, z3::expr> {
 public:
  explicit CachingExprSubstitution(z3::context& c) : src_(c), dst_(c) {}

  z3::context& ctx() { return src_.ctx(); }

  void Clear();

  //! substitute eold for enew
  void AddSubstitution(const z3::expr& eold, const z3::expr& enew);

  //! substitute
  z3::expr operator()(const z3::expr& e);

  // overrides
  z3::expr visitExpr(const z3::expr& e);
  z3::expr visitUNINTERPRETED(const z3::expr&);

 private:
  z3::expr_vector src_, dst_;
  
  using ExprRewriter::Rewrite;
};

//^----------------------------------------------------------------------------^
// helper functions on z3::exprs

inline bool is_not(const z3::expr& e) { return e.decl().decl_kind() == Z3_OP_NOT; }
inline bool is_literal_true(const z3::expr& e) { return Z3_OP_TRUE == e.decl().decl_kind(); }
inline bool is_literal_false(const z3::expr& e) { return Z3_OP_FALSE == e.decl().decl_kind(); }
z3::expr expr_and(const z3::expr& x, const z3::expr& y) {
  if (is_literal_true(x)) {
    return y;
  } else if (is_literal_true(y)) {
    return x;
  } else if (z3::eq(x, y)) {
    return x;
  } else {
    return x && y;
  }
}
z3::expr expr_or(const z3::expr& x, const z3::expr& y) {
  if (is_literal_false(x)) {
    return y;
  } else if (is_literal_false(y)) {
    return x;
  } else if (z3::eq(x, y)) {
    return x;
  } else {
    return x || y;
  }
}

//^----------------------------------------------------------------------------^
// iterators that walk the expr dag
  
struct IterExprSet {
  ExprSet V;
  using iterator = ExprSet::iterator;
  std::pair<iterator,bool> insert(z3::expr e) { return V.insert(e); }
  void completed(z3::expr) {}
};

// Warning: df_expr_iterator was found to be really, really slow under an
// Ubuntu 16.04 linux machine I had. With clang 8. I don't know why. The weird
// part is that df_expr_ext_iterator using IterExprSet -- which should be
// identical to df_expr_iterator -- was much, much faster. *shrug*
//
// This behavior is likely due to the fact that copying this iterator is very
// heavyweight, especially if you're doing it in a loop (e.g., with a
// post-increment). It appears that the standard c++ library assumes iterators
// are *cheap* to copy. LLVM's documentation says to prefer pre-increment
// because many iterators are *expensive* to copy. In any case, we'll prefer
// iterators with *external storage* so this problem doesn't come up very
// often. At least if we can.
using df_expr_iterator     = llvm::df_iterator<z3::expr, IterExprSet>;
using df_expr_ext_iterator = llvm::df_ext_iterator<z3::expr, IterExprSet>;
using po_expr_iterator     = llvm::po_iterator<z3::expr, ExprSet>;
using po_expr_ext_iterator = llvm::po_ext_iterator<z3::expr, ExprSet>;

//^----------------------------------------------------------------------------^

//! Transition system state variable representation
class StateVar {
 public:
  StateVar(std::string name, z3::expr zcurr, z3::expr znext)
      : name_(name),
        zcurr_(zcurr), znext_(znext) {
        assert(z3::eq(zcurr.get_sort(), znext.get_sort()));
      }
  StateVar(const StateVar&) = delete;
  void operator=(const StateVar& x) = delete;
  ~StateVar() = default;

  const std::string& name() const { return name_; }
  const z3::expr& current() const { return zcurr_; }
  const z3::expr& next() const { return znext_; }

  bool operator==(const StateVar& other) const {
    return name_ == other.name_;
  }
  bool operator!=(const StateVar& other) const {
    return !(*this == other);
  }

  //! just hashes based on name
  std::size_t hash() const {
    return std::hash<std::string>()(name_);
  }

 private:
  const std::string name_;
  const z3::expr zcurr_;
  const z3::expr znext_;

};
using StateVarRef = std::reference_wrapper<const StateVar>;

//^----------------------------------------------------------------------------^

//! Transition system primary input representation
class PrimaryInput {
 public:
  PrimaryInput(std::string name, const z3::expr& z) 
      : name_(name), z_(z) {}
  virtual ~PrimaryInput() = default;
  PrimaryInput(const PrimaryInput&) = delete;
  void operator=(const PrimaryInput& x) = delete;

  operator z3::expr() const { return z_; }
  const std::string& name() const { return name_; }

  bool operator==(const PrimaryInput& other) const {
    return name_ == other.name_;
  }
  bool operator!=(const PrimaryInput& other) const {
    return !(*this == other);
  }

  std::size_t hash() const {
    return std::hash<std::string>()(name_);
  }

 private:
  const std::string name_;
  const z3::expr z_;
};
using PrimaryInputRef = std::reference_wrapper<const PrimaryInput>;

//^----------------------------------------------------------------------------^

class TransitionSystem {
 public:
  explicit TransitionSystem(z3::context& ctx)
      : ctx_(ctx), init_state_(ctx.bool_val(true)),
        property_(ctx.bool_val(true)), trans_(ctx.bool_val(true)) {
  }
  void operator=(const TransitionSystem& x) = delete;

  z3::expr init_state() const { return init_state_; }
  void set_init_state(const z3::expr& i) { init_state_ = i; }
  z3::expr property() const { return property_; }
  void set_property(const z3::expr& p) { property_ = p; }
  z3::expr trans() const { return trans_; }
  void set_trans(const z3::expr& t) { trans_ = t; }

  const StateVar& AddVar(std::unique_ptr<StateVar> var) {
    const StateVar& ret = *var.get();
    auto inserted = expr2var_.insert({ret.current(), ref(ret)}).second;
    if (!inserted)
      throw std::runtime_error("duplicate state var");
    inserted = expr2var_.insert({ret.next(), ref(ret)}).second;
    if (!inserted)
      throw std::runtime_error("duplicate state var");
    vars_[var->name()] = move(var);
    return ret;
  }

  inline OptionalRef<const StateVar> FindVar(const std::string& name) const {
    OptionalRef<const StateVar> ret;
    auto search = vars_.find(name);
    if (search != vars_.end()) {
      ret = *search->second;
    }
    return ret;
  }
  
  inline OptionalRef<const StateVar> FindVar(const z3::expr& e) const {
    OptionalRef<const StateVar> ret;
    auto search = expr2var_.find(e);
    if (search != expr2var_.end()) {
      ret = search->second;
    }
    return ret;
  }
  
  inline bool HasCurrentStateVar(const z3::expr& e) const {
    if (auto v = FindVar(e)) {
      return z3::eq(v->current(), e);
    }
    return false;
  }
  
  inline bool HasNextStateVar(const z3::expr& e) const {
    if (auto v = FindVar(e)) {
      return z3::eq(v->next(), e);
    }
    return false;
  }
  
  const PrimaryInput& AddInput(std::unique_ptr<PrimaryInput> inp) {
    const PrimaryInput& ret = *inp.get();
    const z3::expr& ret_expr = *inp.get();
    auto inserted = expr2input_.insert({ret_expr, ref(ret)}).second;
    if (!inserted)
      throw std::runtime_error("duplicate input");
    inputs_[inp->name()] = move(inp);
    return ret;
  }

  inline OptionalRef<const PrimaryInput> 
  FindInput(const std::string& name) const {
    OptionalRef<const PrimaryInput> ret;
    auto search = inputs_.find(name);
    if (search != inputs_.end()) {
      ret = *search->second;
    }
    return ret;
  }
  
  inline OptionalRef<const PrimaryInput> FindInput(const z3::expr& e) const {
    OptionalRef<const PrimaryInput> ret;
    auto search = expr2input_.find(e);
    if (search != expr2input_.end()) {
      ret = search->second;
    }
    return ret;
  }

  size_t num_vars() const { return vars_.size(); }
  size_t num_inputs() const { return inputs_.size(); }

  inline z3::context& ctx() const { return ctx_; }

  //! Returns the same sol_expression with all state variables substituted with
  //! next state versions. Walks the entire sol_expression, so, don't call it
  //! more than you have to.
  z3::expr prime(const z3::expr& e) const;

  z3::expr unprime(const z3::expr& e) const;

  class ConstVarIterator {
   public:
    typedef ConstVarIterator self_type;
    typedef StateVar value_type;
    typedef const StateVar& reference;
    typedef const StateVar *pointer;
    typedef std::forward_iterator_tag iterator_category;
    typedef int difference_type;
    ConstVarIterator(std::unordered_map<std::string,
                     std::shared_ptr<StateVar>>::const_iterator start) 
        : it(start) {}
    inline self_type operator++() { ++it; return *this; }
    inline self_type operator++(int) { self_type i(it); ++*this; return i; }
    inline reference operator*() { return *(*it).second; }
    inline pointer operator->() { return (*it).second.get(); }
    bool operator==(const self_type& rhs) { return it == rhs.it; }
    bool operator!=(const self_type& rhs) { return it != rhs.it; }
   private:
    std::unordered_map<std::string,
        std::shared_ptr<StateVar>>::const_iterator it;
  };
  
  inline ConstVarIterator vbegin() const {
    return ConstVarIterator(std::begin(vars_));
  }
  inline ConstVarIterator vend() const {
    return ConstVarIterator(std::end(vars_));
  }

  boost::iterator_range<ConstVarIterator> vars() const {
    return boost::make_iterator_range(vbegin(), vend());
  }


  class ConstInputIterator {
   public:
    typedef ConstInputIterator self_type;
    typedef PrimaryInput value_type;
    typedef const PrimaryInput& reference;
    typedef const PrimaryInput *pointer;
    typedef std::forward_iterator_tag iterator_category;
    typedef int difference_type;
    ConstInputIterator(std::unordered_map<std::string,
                       std::shared_ptr<PrimaryInput>>::const_iterator start) 
        : it(start) {}
    inline self_type operator++() { ++it; return *this; }
    inline self_type operator++(int) { self_type i(it); ++it; return i; }
    reference operator*() { return *(*it).second; }
    inline pointer operator->() { return (*it).second.get(); }
    bool operator==(const self_type& rhs) { return it == rhs.it; }
    bool operator!=(const self_type& rhs) { return it != rhs.it; }
   private:
    std::unordered_map<std::string,
        std::shared_ptr<PrimaryInput>>::const_iterator it;
  };

  inline ConstInputIterator ibegin() const {
    return ConstInputIterator(std::begin(inputs_));
  }
  inline ConstInputIterator iend() const {
    return ConstInputIterator(std::end(inputs_));
  }
  
  boost::iterator_range<ConstInputIterator> inputs() const {
    return boost::make_iterator_range(ibegin(), iend());
  }

 private:
  z3::context& ctx_;
  z3::expr init_state_;
  z3::expr property_;
  z3::expr trans_;
  // master primary input list
  std::unordered_map<std::string, std::shared_ptr<PrimaryInput>> inputs_;
  // maps input expr to obj
  ExprMap<PrimaryInputRef> expr2input_;
  // master state var list
  std::unordered_map<std::string, std::shared_ptr<StateVar>> vars_;
  // mapss currs_ and nexts_ to state var
  ExprMap<StateVarRef> expr2var_;

  // used to define the substitution in next() and curr()
  std::unique_ptr<CachingExprSubstitution> prime_;
  std::unique_ptr<CachingExprSubstitution> unprime_;
};

std::unique_ptr<StateVar> MakeStateVar(const z3::expr& zcurr,
                                       const z3::expr& znext) {
  assert(zcurr.decl().kind() == znext.decl().kind());
  unique_ptr<StateVar> ret;
  ret = boost::make_unique<StateVar>(
      zcurr.decl().name().str(), zcurr, znext);
  return ret;
}

std::unique_ptr<PrimaryInput> MakeInput(const z3::expr& zvar) {
  unique_ptr<PrimaryInput> ret;
  ret = boost::make_unique<PrimaryInput>(
      zvar.decl().name().str(), zvar);
  return ret;
}
}



namespace llvm {
template <typename SubClass, typename NodeRef, typename Ret,
         typename Set, typename Cache>
class po_iterator_storage<Rewriter<
    SubClass, NodeRef, Ret, Set, Cache>, true> {
 public:
  inline po_iterator_storage(
      Rewriter<SubClass, NodeRef, Ret, Set, Cache>& s)
      : rw_(s) {}

  // Defined below
  void finishPostorder(NodeRef e);
  bool insertEdge(Optional<NodeRef> from, NodeRef to);

 private:
  Rewriter<SubClass, NodeRef, Ret, Set, Cache>& rw_;
};
}


namespace llvm {
template <typename SubClass, typename NodeRef, typename Ret,
         typename Set, typename Cache>
inline void llvm::po_iterator_storage<Rewriter<
    SubClass, NodeRef, Ret, Set, Cache>, true>::finishPostorder(NodeRef e) {
  rw_.FinishPostorder(e);
}

template <typename SubClass, typename NodeRef, typename Ret,
         typename Set, typename Cache>
inline bool llvm::po_iterator_storage<Rewriter<
    SubClass, NodeRef, Ret, Set, Cache>, true>::insertEdge(
        Optional<NodeRef> from, NodeRef to) {
  return rw_.VisitPreorder(from, to);
}

}


namespace {
//! Example: Make a collection c into a conjunction.
// z3::expr conjunction(ctx);
// std::copy(c.begin(), c.end(), ExprAndInserter(conjunction));
// use_conjunction(conjunctien);
class ExprAndInserter {
 public:
   using iterator_category = std::output_iterator_tag;
   using difference_type = void;
   using value_type = void;
   using pointer = void;
   using reference = void;

  inline explicit ExprAndInserter(z3::expr& e) : e_(e) {
    if (!bool(e_)) {
      e_ = e.ctx().bool_val(true);
    }
  }

  inline ExprAndInserter& operator=(const z3::expr& e) {
    e_ = expr_and(e_, e);
    return *this;
  }

  inline ExprAndInserter& operator*() { return *this; }
  inline ExprAndInserter& operator++() { return *this; }
  inline ExprAndInserter& operator++(int) { return *this; }

  z3::expr get() const { return e_; }

 private:
  z3::expr& e_;
};

//! Example: Make a collection c into a disjunction.
// z3::expr disjunction(ctx);
// std::copy(c.begin(), c.end(), ExprAndInserter(disjunction));
//
// Example: Make a cube into a clause.
//
// z3::expr clause(ctx());
// std::transform(cube.begin(), cube.end(), ExprOrInserter(clause), expr_not);
class ExprOrInserter {
 public:
   using iterator_category = std::output_iterator_tag;
   using difference_type = void;
   using value_type = void;
   using pointer = void;
   using reference = void;

  inline explicit ExprOrInserter(z3::expr& e) : e_(e) {
    if (!bool(e_)) {
      e_ = e.ctx().bool_val(false);
    }
  }

  inline ExprOrInserter& operator=(const z3::expr& e) {
    e_ = expr_or(e_, e);
    return *this;
  }

  inline ExprOrInserter& operator*() { return *this; }
  inline ExprOrInserter& operator++() { return *this; }
  inline ExprOrInserter& operator++(int) { return *this; }

  inline z3::expr get() const {
    return e_;
  }

 private:
  z3::expr& e_;
};
}

static Logger logger;

//^----------------------------------------------------------------------------^
// Section 2. Here is the implementation of Horn2VMT.

class HornRule {
 public:
  HornRule(const z3::expr& rule) 
      : head_(rule.ctx()), body_(rule.ctx()), rule_(rule) {
    auto b = rule;
    if (rule.is_quantifier()) {
      b = rule.body();
    } else if (is_not(rule) && rule.arg(0).is_quantifier()) {
      b = rule.arg(0).body();
    }
    body_ = ctx().bool_val(true);
    if (b.is_implies()) {
      head_ = b.arg(1);
      body_ = b.arg(0);
    } else {
      head_ = b;
    }
  }

  z3::context& ctx() const { return head_.ctx(); }
  z3::expr head() const { return head_; }
  z3::expr body() const { return body_; }
  operator z3::expr() const { return rule_; }

 private:
  z3::expr head_;
  z3::expr body_;
  z3::expr rule_;
};

//^----------------------------------------------------------------------------^
//

// Represents a complete set of Horn clauses -- a model checking instance -- as
// a collection of rules. Supports parsing both SMT2-style and
// declare-rel-style Horn clauses, thanks to z3.
class HornClauses {
 public:
  HornClauses(z3::context& c, const char *filename) 
      : ctx_(c), rules_(c) {

    // Reads file into buffer because we may need to parse twice
    std::ifstream fin(filename);
    std::stringstream sstr;
    sstr << fin.rdbuf();
    auto filebuf = sstr.str();

    // Parses as declare-rel at first
    z3::fixedpoint fp(ctx());
    auto vec = Z3_fixedpoint_from_string(c, fp, filebuf.c_str());
    if (!vec) {
      fmt::print(cerr, "Parse error: {}", filename);
      exit(1);
    }
    auto queries = z3::ast_vector_tpl<z3::expr>(c, vec);
    rules_ = fp.rules();
    
    if (queries.size() > 0) {
      assert(queries[0].num_args() == 0);
      rules_.push_back(z3::implies(queries[0], ctx().bool_val(false)));
    }

    if (rules_.empty()) {
      logger.Log(2, "0 fp rules found, so parsing as smtlib2");
      // Parses again as SMTLIB-2 HORN file
      rules_ = ctx().parse_string(filebuf.c_str());
    }
    logger.Log(1, "found {} rules", rules_.size());
  }

  z3::context& ctx() const { return ctx_; }

  z3::ExprVectorIterator rules_begin() const { return z3::begin(rules_); }
  z3::ExprVectorIterator rules_end() const { return z3::end(rules_); }

  class MakeHornRule {
   public:
    HornRule operator()(const z3::expr& rule) const {
      return HornRule(rule);
    }
  };

  // Iterates over the database as HornRules
  boost::iterator_range<
     boost::transform_iterator<MakeHornRule, z3::ExprVectorIterator>>
  rules() const {
    return boost::make_iterator_range(
        boost::make_transform_iterator(rules_begin(), MakeHornRule()),
        boost::make_transform_iterator(rules_end(), MakeHornRule()));
  }
 
 private:
  z3::context& ctx_;
  // Contains all the rules plus the query written as a rule (false => head)
  z3::expr_vector rules_;
};

//^----------------------------------------------------------------------------^
//

class Horn2Vmt {
 public:
  using RelationMap = AstMap<z3::expr>;
  using RelationMapEntry = std::pair<z3::ast, z3::expr>;
  using PlacesMap = AstMap<vector<z3::expr>>;

  Horn2Vmt(const HornClauses&,
           boost::optional<string> varmap_filename);

  void Print() const {
    int64_t index = 0;
    fmt::print("; {} relations\n", relations_.size());
    fmt::print("; {} state vars\n", xsys_.num_vars());
    for (auto&& var : boost::make_iterator_range(xsys_.vbegin(), xsys_.vend())) {
      fmt::print("{}\n", var.current().decl());
      fmt::print("{}\n", var.next().decl());
      fmt::print("(define-fun .def{} () {} (! {} :next {}))\n", index,
                 var.current().get_sort(), var.current(),
                 var.next());
      ++index;
    }
    fmt::print("; {} inputs\n", xsys_.num_inputs());
    for (const z3::expr&& input : boost::make_iterator_range(xsys_.ibegin(), xsys_.iend())) {
      fmt::print("{}\n", input.decl());
    }
    fmt::print("(define-fun .def{} () Bool (! {} :init true))\n", index++,
               xsys_.init_state());
    fmt::print("(define-fun .def{} () Bool (! {} :trans true))\n", index++,
               xsys_.trans());
    fmt::print("(define-fun .def{} () Bool (! {} :invar-property 0))\n", index,
               xsys_.property());
  }


  z3::context& ctx() const { return hc_.ctx(); }

 private:
  const HornClauses& hc_;
  TransitionSystem xsys_;
  // Maps func_decl to Boolean state var
  RelationMap relations_;
  // Maps relation Bool to list of place state vars
  PlacesMap relation_places_;
  // Maps rule index to quantifier var place inputs
  vector<vector<z3::expr>> rule_places_;
  // Used by FillVarSubst for creating unique inputs
  int64_t counter_;

  const bool allocate_inputs_by_sort_ = true;
  boost::optional<string> varmap_filename_;

  void CreateStateSpace();
  void FillVarSubst(const HornRule&, int index, ExprSubstitution *var_subst_curr);
  z3::expr CreateRuleCondition(const z3::expr& e);
  void CreateHeadEquations(const z3::expr& ra, const z3::expr&);

  void CheckHornRuleIsLinear(const HornRule& rule) {
    IterExprSet visited;
    int relation_count =
        count_if(df_expr_ext_iterator::begin(rule.body(), visited),
                 df_expr_ext_iterator::end(rule.body(), visited),
                 [&](const z3::expr& x) { return x.is_app() &&
                     relations_.find(x.decl()) != relations_.end(); });
    if (relation_count > 1) {
      throw std::runtime_error("nonlinear Horn clause detected, exiting");
    }
  }
};
  
Horn2Vmt::Horn2Vmt(const HornClauses& hc, boost::optional<string> vm) 
    : hc_(hc), xsys_(hc.ctx()), counter_(0),
      varmap_filename_(vm) {
  using namespace boost::adaptors; // indexed
  bool property_set = false;
  // Pass over entire database
  //
  // 1. Creates relation Boolean vars
  // 2. Creates place sorted vars
  // 3. Creates rule Boolean inputs
  //
  // This will create all the state variables that the transition system
  // needs. Fiils in relations_, relation_places_, rule_places, rule_activations_
  CreateStateSpace();

  // This will be incrementally added to below
  z3::expr trans(ctx());
  auto trans_out = ExprOrInserter(trans);

  // Translates the rules one at a time with several passes over the body and
  // head. I took care to make sure the I do several linear passes instead of
  // one superlinear one.
  for (const auto& indexed_rule : hc_.rules() | indexed(0)) {
    z3::expr transition(ctx());
    auto transition_out = ExprAndInserter(transition);
    const auto& rule = indexed_rule.value();
    logger.Log(1, "\n********************************************************"
               "**********************\nProcessing rule {}: {} => {}",
               indexed_rule.index(), rule.body(), rule.head());
    // Turn off for now.
    // CheckHornRuleIsLinear(rule);
    ExprSubstitution var_subst_curr(ctx());
    FillVarSubst(rule, indexed_rule.index(), &var_subst_curr);
    // Translates the rule's body into a global condition on inputs & *bound*
    // vars
    auto body_condition = CreateRuleCondition(rule.body());
    logger.Log(3, "Rule {} condition from Horn:\n{}", indexed_rule.index(),
               body_condition);
    body_condition = var_subst_curr(body_condition);
    // Adds body condition to this transition
    *transition_out++ = body_condition;

    // set of modified next-state vars
    ExprSet mod;
    // The code below adds next-state constraints to this transition based on
    // the rule head
    const auto head_decl = rule.head().decl();
    if (head_decl.decl_kind() == Z3_OP_UNINTERPRETED) {
      const auto relation_var = xsys_.FindVar(relations_.at(head_decl));
      mod.insert(relation_var->next());
      for (auto&& zipped : boost::combine(ExprArgs(rule.head()),
                                          relation_places_.at(head_decl))) {
        z3::expr arg(ctx()), place(ctx());
        boost::tie(arg, place) = zipped;
        auto place_var = xsys_.FindVar(place);
        mod.insert(place_var->next());
        *transition_out++ = (place_var->next() == var_subst_curr(arg));
      }
      // Adds var preservation constraints
      for (const auto& var : xsys_.vars()) {
        if (mod.find(var.next()) == mod.end()) {
          *transition_out++ = (var.next() == var.current());
        }
      }
      // Adds this transition to the transition relation
      *trans_out++ = transition_out.get();
    } else {
      if (property_set) {
        FATAL("found multiple properties, exiting:\n"
                      "previous property: {}\n"
                      "current rule head: {}\n",
                      xsys_.property(), rule.head());
      }
      if (is_not(rule.head()) &&
          rule.head().arg(0).decl().decl_kind() == Z3_OP_UNINTERPRETED) {
        // property is denoted by (assert !R)
        xsys_.set_property(CreateRuleCondition(rule.head()));
        property_set = true;
      } else if (z3::eq(rule.head(), ctx().bool_val(false))) {
        // property is denoted by (R => false)
        xsys_.set_property(!body_condition);
        property_set = true;
      } else {
        FATAL("unrecognized rule: {} => {}", rule.body(), rule.head());
      }
    }
  }
  if (!property_set) {
    fmt::print(std::cerr, "horn2vmt: no property found in Horn clauses, exiting\n");
    return;
  }

  xsys_.set_trans(trans_out.get());

  // In the initial state, all relations are false.
  z3::expr init(ctx());
  transform(relations_.begin(), relations_.end(), ExprAndInserter(init),
            [&](Horn2Vmt::RelationMapEntry&& p) { return !xsys_.FindVar(p.second)->current(); });
  xsys_.set_init_state(init);

}

static string SortName(const z3::sort& sort) {
  if (sort.is_bv()) {
    return fmt::format("bv{}", sort.bv_size());
  } else if (sort.is_bool()) {
    return "bool";
  } else if (sort.is_array()) {
    auto domain_name = SortName(sort.array_domain());
    auto range_name = SortName(sort.array_range());
    return fmt::format("arr-{}-{}", domain_name, range_name);
  } else if (sort.is_int()) {
    return "int";
  } else {
    fmt::print("{}\n", sort);
    FATAL("unhandled sort");
  }
}

// Fills in relations_, relation_places_, rule_places_, and rule_activations_
void Horn2Vmt::CreateStateSpace() {
  using namespace boost::adaptors; // indexed
  
  SortMap<std::vector<z3::expr>> relation_places_by_sort;
  SortMap<std::vector<z3::expr>> rule_places_by_sort;
  for (const auto& indexed_rule : hc_.rules() | indexed(0)) {
    const auto& rule = indexed_rule.value();
    const auto index = indexed_rule.index();
    logger.Log(3, "creating state space for rule: {} => {}", rule.body(),
               rule.head());

    // Makes any inputs required to express the quantified variables in the
    // rule, binning them by sort
    rule_places_.push_back(vector<z3::expr>());
    assert(static_cast<int>(rule_places_.size()) == index+1);
    z3::expr rule_expr = rule;
    if (rule_expr.is_quantifier()) {
      const unsigned num_bound_vars =
          Z3_get_quantifier_num_bound(rule_expr.ctx(), rule_expr); 
      if (allocate_inputs_by_sort_) {
        SortMap<int> counts;
        for (unsigned i = 0; i < num_bound_vars; i++) {
          z3::sort sort(
              rule_expr.ctx(),
              Z3_get_quantifier_bound_sort(rule_expr.ctx(), rule_expr, i));
          if (counts.find(sort) == counts.end()) {
            counts[sort] = 0;
          }
          counts[sort]++;
        }
        for (std::pair<z3::sort, int>&& entry : counts) {
          auto sort = entry.first;
          auto num_occurrences = entry.second;
          int num_places =
              static_cast<int>(rule_places_by_sort[sort].size());
          for (int i = 0; i < (num_occurrences - num_places); i++) {
            auto name =
                (SortName(sort) + "-input-" + to_string(num_places+i));
            auto& input_var = xsys_.AddInput(MakeInput(
                    ctx().constant(name.c_str(), sort)));
            rule_places_by_sort[sort].push_back(input_var);
          }
        }
        counts.clear();
        for (unsigned i = 0; i < num_bound_vars; i++) {
          z3::sort sort(
              rule_expr.ctx(),
              Z3_get_quantifier_bound_sort(rule_expr.ctx(), rule_expr, i));
          auto var = rule_places_by_sort.at(sort)[counts[sort]];
          rule_places_[index].push_back(var);
          counts[sort]++;
        }
        // Apparently the de-bruijn indices of variables are the revers from the
        // way they would be in a list...there's some verbiage to this effect in
        // the constructor for forall's in the Z3 documentation but I didn't
        // realize it would extend to this API. I guess it does. 
        std::reverse(rule_places_[index].begin(), rule_places_[index].end());
      } else {
        // Create inputs named input-<rule-number>-<deBruijn index>
        for (unsigned i = 0; i < num_bound_vars; i++) {
          z3::sort sort(
              rule_expr.ctx(),
              Z3_get_quantifier_bound_sort(rule_expr.ctx(), rule_expr, i));
          z3::symbol var_name(
              rule_expr.ctx(),
              Z3_get_quantifier_bound_name(rule_expr.ctx(), rule_expr, i));
          auto name = fmt::format("input-{}-{}", index, var_name.str());
          auto& input_var = xsys_.AddInput(
              MakeInput(ctx().constant(name.c_str(), sort)));
          rule_places_[index].push_back(input_var);
        }
        std::reverse(rule_places_[index].begin(), rule_places_[index].end());
      }
    }
    // Pick up relations from rule heads; then declare Boolean relation vars
    // and place vars for each place. 
    auto decl = rule.head().decl();
    if (decl.decl_kind() == Z3_OP_UNINTERPRETED) {
      // Create r elation var
      auto name = decl.name().str();
      auto var_name = "at-" + name;
      if (xsys_.FindVar(var_name)) { // Skips relations occurring in > 1 rules
        continue;
      }
      auto& var = xsys_.AddVar(MakeStateVar(
              ctx().bool_const(var_name.c_str()),
              ctx().bool_const((var_name + "+").c_str())));
      relations_.insert({decl, var.current()});
      // Ensures vector is initialized for all relations, even nullary ones
      (void)relation_places_[decl];
      // Open varmap file, if we need to
      ofstream varmap_out;
      if (varmap_filename_) {
        varmap_out = ofstream(*varmap_filename_);
        if (!varmap_out) {
          fmt::print(cerr, "warning: could not open varmap file: {}:\n{}\n",
                     *varmap_filename_, strerror(errno));
          varmap_filename_ = boost::none;
        }
      }
      for (unsigned i = 0; i < decl.arity(); i++) {
        auto& place_var = xsys_.AddVar(MakeStateVar(
                ctx().constant((name + "-place-" + to_string(i)).c_str(),
                               decl.domain(i)),
                ctx().constant((name + "-place-" + to_string(i) + "+").c_str(),
                               decl.domain(i))));
        if (varmap_out) {
          fmt::print(varmap_out, "Relation {} place {} mapped to var {}",
                     name, i, place_var.current());
        }
        relation_places_[decl].push_back(place_var.current());
      }
    }
  }
  logger.Log(3, "created {} state variables", xsys_.num_vars());
  if (logger.ShouldLog(4)) {
    for (auto&& v : boost::make_iterator_range(xsys_.vbegin(), xsys_.vend())) {
      logger.Log(4, "  {}", v.current());
    }
  }
  logger.Log(3, "created {} inputs", xsys_.num_inputs());
}


void Horn2Vmt::FillVarSubst(const HornRule& rule, int index,
                            ExprSubstitution *var_subst_curr) {
  // Set of all visited nodes for df traversal
  IterExprSet visited;
  // Precondition: e is a sub-expression of rule
  auto visit_create_input = [&](const z3::expr& e) {
    if (e.is_var()) {
      auto input = rule_places_[index][Z3_get_index_value(e.ctx(), e)];
      var_subst_curr->AddSubstitution(e, input);
      logger.Log(3, "adding input [{} / {}] to var_subst_curr", e, input);
    }
  };
  
  auto rule_expr = rule.body() && rule.head();
  for_each(df_expr_ext_iterator::begin(rule_expr, visited),
           df_expr_ext_iterator::end(rule_expr, visited), visit_create_input);
}


class HornBodyRewriter : public ExprRewriter<HornBodyRewriter> {
 public:
  HornBodyRewriter(const Horn2Vmt::RelationMap& r,
                   const Horn2Vmt::PlacesMap& p) : relations_(r), relation_places_(p) {}

  z3::expr operator()(const z3::expr& e) { return Rewrite(e); }

  // Precondition: e is a sub-formula of a Horn clause body.
  // If e is a relation occurrence, rewrites it in T-land using the relation
  // and place variables. Otherwise, simply rewrites e using previously
  // rewritten children.
  z3::expr visit(const z3::expr& e) {
    if (e.is_app()) {
      auto search = relations_.find(e.decl());
      if (search != relations_.end()) {
        z3::expr conj(e.ctx());
        ExprAndInserter out(conj);
        *out++ = search->second;
        for (auto&& zipped : boost::combine(ExprArgs(e),
                                            relation_places_.at(e.decl()))) {
          z3::expr arg(e.ctx()), place_var(e.ctx());
          boost::tie(arg, place_var) = zipped;
          *out++ = (arg == place_var);
        }
        return out.get();
      }
      return e.decl()(NewArgsExprVector(e));
    }
    return e;
  }

 private:
  const Horn2Vmt::RelationMap& relations_;
  const Horn2Vmt::PlacesMap& relation_places_;
};

z3::expr Horn2Vmt::CreateRuleCondition(const z3::expr& body) {
  logger.Log(3, "creating rule condition for body", body);
  HornBodyRewriter to_vmt(relations_, relation_places_);
  return to_vmt(body);
}



//^----------------------------------------------------------------------------^
//
namespace po = boost::program_options;

po::options_description desc("options");

int main(int argc, char **argv) {
  // option parsing
  desc.add_options()
      ("help,h", "help")
      ("v", po::value<int>(), "set log level")
      ("varmap", po::value<string>(), "output mapping between relation places and state variables to this file")
      ("filename", po::value<string>(), "SMT2 file to convert");
  po::positional_options_description pd;
  pd.add("filename", -1);
  po::variables_map vmap;
  po::store(po::command_line_parser(argc, argv).
            options(desc).positional(pd).run(), vmap);
  po::notify(vmap);

  if (!vmap.count("filename")) {
    fmt::print(cerr, "error: missing filename\n");
  }
  if (vmap.count("help") || !vmap.count("filename")) {
    fmt::print(cout, "{} [options] filename\n", argv[0]);
    fmt::print(cout, "{}\n", desc);
    return EXIT_FAILURE;
  }

  logger.set_level(0);
  if (vmap.count("v")) {
    logger.set_level(vmap["v"].as<int>());
  }

  boost::optional<string> varmap_filename;
  if (vmap.count("varmap")) {
    varmap_filename = string(vmap["varmap"].as<string>());
  }

  z3::context ctx;
  auto filename = vmap["filename"].as<string>();
  HornClauses clause_db(ctx, filename.c_str());
  Horn2Vmt h2v(clause_db, varmap_filename);
  h2v.Print();

  return EXIT_SUCCESS;
}
