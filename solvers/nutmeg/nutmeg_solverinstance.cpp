/* -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil -*- */

/*
 *  Main authors:
 *     Edward Lam <edward.lam@monash.edu>
 *     Jip J. Dekker <jip.dekker@monash.edu>
 */

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <minizinc/solvers/nutmeg_solverinstance.hh>
#include <minizinc/solvers/nutmeg/nutmeg_constraints.hh>

using namespace Nutmeg;

namespace MiniZinc{
  NutmegSolverInstance::NutmegSolverInstance(Env &env, std::ostream &log, SolverInstanceBase::Options *opt)
      : SolverInstanceImpl<NutmegTypes>(env, log, opt), _flat(env.flat()),
        _solver(Nutmeg::Method::BC) {
    registerConstraints();
  }

  void NutmegSolverInstance::registerConstraint(std::string name, poster p) {
    _constraintRegistry.add("nutmeg_" + name, p);
    _constraintRegistry.add(name, p);
  }

  void NutmegSolverInstance::registerConstraints() {
    GCLock lock;
    /* Integer Comparison Constraints */
    registerConstraint("int_eq", NutmegConstraints::p_int_eq);
//     registerConstraint("int_ne", NutmegConstraints::p_int_ne);
     registerConstraint("int_le", NutmegConstraints::p_int_le);
     registerConstraint("int_lt", NutmegConstraints::p_int_lt);
//     registerConstraint("int_eq_imp", NutmegConstraints::p_int_eq_imp);
//     registerConstraint("int_ne_imp", NutmegConstraints::p_int_ne_imp);
//     registerConstraint("int_le_imp", NutmegConstraints::p_int_le_imp);
//     registerConstraint("int_lt_imp", NutmegConstraints::p_int_lt_imp);
//     registerConstraint("int_eq_reif", NutmegConstraints::p_int_eq_reif);
//     registerConstraint("int_ne_reif", NutmegConstraints::p_int_ne_reif);
//     registerConstraint("int_le_reif", NutmegConstraints::p_int_le_reif);
//     registerConstraint("int_lt_reif", NutmegConstraints::p_int_lt_reif);

     /* Integer Arithmetic Constraints */
//     registerConstraint("int_abs", NutmegConstraints::p_int_abs);
//     registerConstraint("int_times", NutmegConstraints::p_int_times);
//     registerConstraint("int_div", NutmegConstraints::p_int_div);
// //    registerConstraint("int_mod", NutmegConstraints::p_int_mod);
//     registerConstraint("int_min", NutmegConstraints::p_int_min);
//     registerConstraint("int_max", NutmegConstraints::p_int_max);

     /* Integer Linear Constraints */
     registerConstraint("int_lin_eq", NutmegConstraints::p_int_lin_eq);
     registerConstraint("int_lin_ne", NutmegConstraints::p_int_lin_ne);
     registerConstraint("int_lin_le", NutmegConstraints::p_int_lin_le);
//     registerConstraint("int_lin_eq_imp", NutmegConstraints::p_int_lin_eq_imp);
//     registerConstraint("int_lin_ne_imp", NutmegConstraints::p_int_lin_ne_imp);
//     registerConstraint("int_lin_le_imp", NutmegConstraints::p_int_lin_le_imp);
//     registerConstraint("int_lin_eq_reif", NutmegConstraints::p_int_lin_eq_reif);
//     registerConstraint("int_lin_ne_reif", NutmegConstraints::p_int_lin_ne_reif);
//     registerConstraint("int_lin_le_reif", NutmegConstraints::p_int_lin_le_reif);

     /* Boolean Comparison Constraints */
//     registerConstraint("bool_eq", NutmegConstraints::p_bool_eq);
//     registerConstraint("bool_ne", NutmegConstraints::p_bool_ne);
//     registerConstraint("bool_le", NutmegConstraints::p_bool_le);
//     registerConstraint("bool_lt", NutmegConstraints::p_bool_lt);
//     registerConstraint("bool_eq_imp", NutmegConstraints::p_bool_eq_imp);
//     registerConstraint("bool_ne_imp", NutmegConstraints::p_bool_ne_imp);
//     registerConstraint("bool_le_imp", NutmegConstraints::p_bool_le_imp);
//     registerConstraint("bool_lt_imp", NutmegConstraints::p_bool_lt_imp);
//     registerConstraint("bool_eq_reif", NutmegConstraints::p_bool_eq_reif);
//     registerConstraint("bool_ne_reif", NutmegConstraints::p_bool_ne_reif);
//     registerConstraint("bool_le_reif", NutmegConstraints::p_bool_le_reif);
//     registerConstraint("bool_lt_reif", NutmegConstraints::p_bool_lt_reif);

     /* Boolean Arithmetic Constraints */
//     registerConstraint("bool_or", NutmegConstraints::p_bool_or);
//     registerConstraint("bool_and", NutmegConstraints::p_bool_and);
//     registerConstraint("bool_xor", NutmegConstraints::p_bool_xor);
//     registerConstraint("bool_not", NutmegConstraints::p_bool_not);
//     registerConstraint("bool_or_imp", NutmegConstraints::p_bool_or_imp);
//     registerConstraint("bool_and_imp", NutmegConstraints::p_bool_and_imp);
//     registerConstraint("bool_xor_imp", NutmegConstraints::p_bool_xor_imp);

//     registerConstraint("bool_clause", NutmegConstraints::p_bool_clause);
//     registerConstraint("array_bool_or", NutmegConstraints::p_array_bool_or);
//     registerConstraint("array_bool_and", NutmegConstraints::p_array_bool_and);
//     registerConstraint("bool_clause_imp", NutmegConstraints::p_bool_clause_imp);
//     registerConstraint("array_bool_or_imp", NutmegConstraints::p_array_bool_or_imp);
//     registerConstraint("array_bool_and_imp", NutmegConstraints::p_array_bool_and_imp);
//     registerConstraint("bool_clause_reif", NutmegConstraints::p_bool_clause_reif);

     /* Boolean Linear Constraints */
//     registerConstraint("bool_lin_eq", NutmegConstraints::p_bool_lin_eq);
//     registerConstraint("bool_lin_ne", NutmegConstraints::p_bool_lin_ne);
//     registerConstraint("bool_lin_le", NutmegConstraints::p_bool_lin_le);
//     registerConstraint("bool_lin_eq_imp", NutmegConstraints::p_bool_lin_eq_imp);
//     registerConstraint("bool_lin_ne_imp", NutmegConstraints::p_bool_lin_ne_imp);
//     registerConstraint("bool_lin_le_imp", NutmegConstraints::p_bool_lin_le_imp);
//     registerConstraint("bool_lin_eq_reif", NutmegConstraints::p_bool_lin_eq_reif);
//     registerConstraint("bool_lin_ne_reif", NutmegConstraints::p_bool_lin_ne_reif);
//     registerConstraint("bool_lin_le_reif", NutmegConstraints::p_bool_lin_le_reif);

     /* Coercion Constraints */
//     registerConstraint("bool2int", NutmegConstraints::p_bool2int);

     /* Element Constraints */
     registerConstraint("array_int_element", NutmegConstraints::p_array_int_element);
//     registerConstraint("array_bool_element", NutmegConstraints::p_array_bool_element);
     registerConstraint("array_var_int_element", NutmegConstraints::p_array_var_int_element);
//     registerConstraint("array_var_bool_element", NutmegConstraints::p_array_var_bool_element);

     /* Global Constraints */
     registerConstraint("all_different_int", NutmegConstraints::p_all_different);
//     registerConstraint("alldifferent_except_0", NutmegConstraints::p_all_different_except_0);
//     registerConstraint("at_most", NutmegConstraints::p_at_most);
//     registerConstraint("at_most1", NutmegConstraints::p_at_most1);
//     registerConstraint("cumulative", NutmegConstraints::p_cumulative);
//     registerConstraint("cumulative_var", NutmegConstraints::p_cumulative);
//     registerConstraint("disjunctive", NutmegConstraints::p_disjunctive);
//     registerConstraint("disjunctive_var", NutmegConstraints::p_disjunctive);
//     registerConstraint("global_cardinality", NutmegConstraints::p_global_cardinality);
//     registerConstraint("table_int", NutmegConstraints::p_table_int);

    /**** TODO: NOT YET SUPPORTED: ****/
    /* Boolean Arithmetic Constraints */
//    registerConstraint("array_bool_xor", NutmegConstraints::p_array_bool_xor);
//    registerConstraint("array_bool_xor_imp", NutmegConstraints::p_array_bool_xor_imp);

    /* Floating Point Comparison Constraints */
//    registerConstraint("float_eq", NutmegConstraints::p_float_eq);
//    registerConstraint("float_le", NutmegConstraints::p_float_le);
//    registerConstraint("float_lt", NutmegConstraints::p_float_lt);
//    registerConstraint("float_ne", NutmegConstraints::p_float_ne);
//    registerConstraint("float_eq_reif", NutmegConstraints::p_float_eq_reif);
//    registerConstraint("float_le_reif", NutmegConstraints::p_float_le_reif);
//    registerConstraint("float_lt_reif", NutmegConstraints::p_float_lt_reif);

    /* Floating Point Arithmetic Constraints */
//    registerConstraint("float_abs", NutmegConstraints::p_float_abs);
//    registerConstraint("float_sqrt", NutmegConstraints::p_float_sqrt);
//    registerConstraint("float_times", NutmegConstraints::p_float_times);
//    registerConstraint("float_div", NutmegConstraints::p_float_div);
//    registerConstraint("float_plus", NutmegConstraints::p_float_plus);
//    registerConstraint("float_max", NutmegConstraints::p_float_max);
//    registerConstraint("float_min", NutmegConstraints::p_float_min);
//    registerConstraint("float_acos", NutmegConstraints::p_float_acos);
//    registerConstraint("float_asin", NutmegConstraints::p_float_asin);
//    registerConstraint("float_atan", NutmegConstraints::p_float_atan);
//    registerConstraint("float_cos", NutmegConstraints::p_float_cos);
//    registerConstraint("float_exp", NutmegConstraints::p_float_exp);
//    registerConstraint("float_ln", NutmegConstraints::p_float_ln);
//    registerConstraint("float_log10", NutmegConstraints::p_float_log10);
//    registerConstraint("float_log2", NutmegConstraints::p_float_log2);
//    registerConstraint("float_sin", NutmegConstraints::p_float_sin);
//    registerConstraint("float_tan", NutmegConstraints::p_float_tan);

    /* Floating Linear Constraints */
//    registerConstraint("float_lin_eq", NutmegConstraints::p_float_lin_eq);
//    registerConstraint("float_lin_eq_reif", NutmegConstraints::p_float_lin_eq_reif);
//    registerConstraint("float_lin_le", NutmegConstraints::p_float_lin_le);
//    registerConstraint("float_lin_le_reif", NutmegConstraints::p_float_lin_le_reif);

    /* Coercion Constraints */
//    registerConstraint("int2float", NutmegConstraints::p_int2float);
  }

  void NutmegSolverInstance::processFlatZinc() {

      // Get options.
      auto _opt = static_cast<NutmegOptions&>(*_options);

      // Create variables.
      for (auto it = _flat->begin_vardecls(); it != _flat->end_vardecls(); ++it) {
          if (!it->removed() && it->e()->type().isvar() && it->e()->type().dim() == 0) {
              VarDecl* vd = it->e();

              if (vd->type().isbool()) {
                  if (!vd->e()) {
                      Expression* domain = vd->ti()->domain();
                      long long int lb, ub;
                      if (domain) {
                          IntBounds ib = compute_int_bounds(_env.envi(), domain);
                          lb = ib.l.toInt();
                          ub = ib.u.toInt();
                      } else {
                          lb = 0;
                          ub = 1;
                      }
                      if (lb == ub) {
                          BoolVar val = (lb == 0) ? _solver.get_false() : _solver.get_true();
                          _variableMap.insert(vd->id(), NutmegVariable(val));
                      } else {
                          const auto& name = vd->id()->str().str();
                          auto var = _solver.add_bool_var(name);
                          _variableMap.insert(vd->id(), NutmegVariable(var));
                      }
                  } else {
                      Expression* init = vd->e();
                      if (init->isa<Id>() || init->isa<ArrayAccess>()) {
                          NutmegVariable& var = resolveVar(init);
                          assert(var.isBool());
                          _variableMap.insert(vd->id(), NutmegVariable(var.boolVar()));
                      } else {
                          auto b = init->cast<BoolLit>()->v();
                          BoolVar val = b ? _solver.get_true() : _solver.get_false();
                          _variableMap.insert(vd->id(), NutmegVariable(val));
                      }
                  }
              } else if (vd->type().isint()) {
                  if (!vd->e()) {
                      Expression* domain = vd->ti()->domain();
                      if (domain) {
                          IntSetVal* isv = eval_intset(env().envi(), domain);

                          const auto min = static_cast<Int>(isv->min().toInt());
                          const auto max = static_cast<Int>(isv->max().toInt());
                          const auto& name = vd->id()->str().str();

                          IntVar var;
                          var = _solver.add_int_var(min, max, true, name); // TODO true
                          if (isv->size() > 1) {
//                              Vector<Int> vals(static_cast<int>(isv->card().toInt())); // TODO A
                              Vector<Int> vals;
                              int i = 0;
                              for (int j = 0; j < isv->size(); ++j) {
                                  for (auto k = isv->min(i).toInt(); k <= isv->max(j).toInt(); ++k) {
                                      vals.push_back(static_cast<int>(k));
//                                      vals[i++] = static_cast<int>(k); // TODO A
                                  }
                              }
//                              assert(i == isv->card().toInt()); TODO A
                              _solver.add_indicator_vars(var, vals);
                          }
                          _variableMap.insert(vd->id(), NutmegVariable(var));
                      } else {
                          throw Error("NutmegSolverInstance::processFlatZinc: Error: Unbounded variable: " + vd->id()->str().str());
                      }
                  } else {
                      Expression* init = vd->e();
                      if (init->isa<Id>() || init->isa<ArrayAccess>()) {
                          NutmegVariable& var = resolveVar(init);
                          assert(var.isInt());
                          _variableMap.insert(vd->id(), NutmegVariable(var.intVar()));
                      } else {
                          auto il = init->cast<IntLit>()->v().toInt();

                          const auto constant = static_cast<Nutmeg::Int>(il);
                          auto var = _solver.add_int_var(constant, constant, true, fmt::format("{}", constant));

                          _variableMap.insert(vd->id(), NutmegVariable(var));
                      }
                  }
              } else {
                  std::stringstream ssm;
                  ssm << "Type " << *vd->ti() << " is currently not supported by Nutmeg.";
                  throw InternalError(ssm.str());
              }
          }
      }

      // Post constraints.
      for (ConstraintIterator it = _flat->begin_constraints(); it != _flat->end_constraints(); ++it) {
          if(!it->removed()) {
              if (auto c = it->e()->dyn_cast<Call>()) {
                  _constraintRegistry.post(c);
              }
          }
      }

      // Set objective.
      SolveI* si = _flat->solveItem();
      if(si->e()) {
          _obj_type = si->st();
          if (_obj_type == SolveI::ST_MIN) {
              _obj_var = std::unique_ptr<NutmegTypes::Variable>(new NutmegTypes::Variable(resolveVar(si->e())));
          } else if (_obj_type == SolveI::ST_MAX) {
              _obj_type = SolveI::ST_MIN;

              const auto original_var = asIntVar(si->e());
              const auto var = solver().add_int_var(-solver().ub(original_var),
                                                    -solver().lb(original_var),
                                                    true,
                                                    "-" + solver().name(original_var));
              _obj_var = std::unique_ptr<NutmegTypes::Variable>(new NutmegTypes::Variable(var));

              release_assert(solver().add_constr_linear({original_var, var}, {1, 1}, Sign::EQ, 0),
                             "Failed to create objective variable for maximization problem");
          }
      }
      if (!si->ann().isEmpty()) {
          println("Warning: Nutmeg does not support search annotations");
      }
  }

  SolverInstanceBase::Status MiniZinc::NutmegSolverInstance::solve() {
      SolverInstanceBase::Status status = SolverInstance::ERROR;
      auto _opt = static_cast<NutmegOptions&>(*_options);
      const auto time_limit = _opt.time == std::chrono::milliseconds(0) ? Nutmeg::Infinity : _opt.time.count() / 1e3;

      if (_obj_type == SolveI::ST_SAT)
      {
          not_yet_implemented();
//      int nr_solutions = 0;
//      geas::solver::result res = geas::solver::UNKNOWN;
//      while ((_opt.all_solutions || nr_solutions < _opt.nr_solutions) && remaining_time() >= 0.0) {
//        res = _solver.solve({remaining_time(), _opt.conflicts - _solver.data->stats.conflicts});
//        nr_solutions++;
//        printSolution();
//        if (res != geas::solver::SAT) {
//          break;
//        } else {
//          _solver.restart();
//          if(!addSolutionNoGood()) {
//            res = geas::solver::UNSAT;
//            break;
//          }
//        }
//      }
//      switch (res) {
//        case geas::solver::SAT:
//          status = SolverInstance::SAT;
//          break;
//        case geas::solver::UNSAT:
//          if (nr_solutions > 0) {
//            status = SolverInstance::OPT;
//          } else {
//            status = SolverInstance::UNSAT;
//          }
//          break;
//        case geas::solver::UNKNOWN:
//          if (nr_solutions > 0) {
//            status = SolverInstance::SAT;
//          } else {
//            status = SolverInstance::UNKNOWN;
//          }
//          break;
//        default:
//          status = SolverInstance::ERROR;
//          break;
//      }
      } else {
          assert(_obj_type == SolveI::ST_MIN);
          assert(_obj_var->isInt());

          _solver.minimize(_obj_var->intVar(), time_limit);
          const auto solver_status = _solver.get_status();

          switch (solver_status)
          {
              case Nutmeg::Status::Error:
                  assert(false);
                  status = SolverInstance::ERROR;
                  break;
              case Nutmeg::Status::Infeasible:
                  status = SolverInstance::UNSAT;
                  break;
              case Nutmeg::Status::Unknown:
                  status = SolverInstance::UNKNOWN;
                  break;
              case Nutmeg::Status::Optimal:
                  status = SolverInstance::OPT;
              case Nutmeg::Status::Feasible:
                  status = SolverInstance::SAT;

                  if (!_opt.all_solutions)
                  {
                      printSolution();
                  }

                  break;
          }
      }
      if (_opt.statistics) {
          printStatistics(true);
      }
      return status;
  }

  Expression* NutmegSolverInstance::getSolutionValue(Id* id) {
    id = id->decl()->id();
    if(id->type().isvar()) {
      NutmegVariable& var = resolveVar(id->decl()->id());
      switch (id->type().bt()) {
        case Type::BT_BOOL:
          assert(var.isBool());
          return constants().boollit(_solver.get_sol(var.boolVar()));
        case Type::BT_INT:
          assert(var.isInt());
          return IntLit::a(_solver.get_sol(var.intVar()));
        default:
          return nullptr;
      }
    } else {
      return id->decl()->e();
    }
  }

  void NutmegSolverInstance::resetSolver() {
      not_yet_implemented();
  }

  NutmegTypes::Variable& NutmegSolverInstance::resolveVar(Expression* e) {
    if (auto id = e->dyn_cast<Id>()) {
      return _variableMap.get(id->decl()->id());
    } else if (auto vd = e->dyn_cast<VarDecl>()) {
      return _variableMap.get(vd->id()->decl()->id());
    } else if (auto aa = e->dyn_cast<ArrayAccess>()) {
      auto ad = aa->v()->cast<Id>()->decl();
      auto idx = aa->idx()[0]->cast<IntLit>()->v().toInt();
      auto al = eval_array_lit(_env.envi(), ad->e());
      return _variableMap.get((*al)[idx]->cast<Id>());
    } else {
      std::stringstream ssm;
      ssm << "Expected Id, VarDecl or ArrayAccess instead of \"" << *e << "\"";
      throw InternalError(ssm.str());
    }
  }

  Nutmeg::Vector<bool> NutmegSolverInstance::asBool(ArrayLit* al) {
    Nutmeg::Vector<bool> vec(al->size());
    for (int i = 0; i < al->size(); ++i) {
      vec[i] = asBool((*al)[i]);
    }
    return vec;
  }

  Nutmeg::BoolVar NutmegSolverInstance::asBoolVar(Expression* e) {
    if (e->type().isvar()) {
      NutmegVariable& var = resolveVar(follow_id_to_decl(e));
      assert(var.isBool());
      return var.boolVar();
    } else {
      if(auto bl = e->dyn_cast<BoolLit>()) {
        return bl->v() ? _solver.get_true() : _solver.get_false();
      } else {
        std::stringstream ssm; ssm << "Expected bool or int literal instead of: " << *e;
        throw InternalError(ssm.str());
      }
    }
  }

  Nutmeg::Vector<Nutmeg::BoolVar> NutmegSolverInstance::asBoolVar(ArrayLit* al) {
    Nutmeg::Vector<Nutmeg::BoolVar> vec(al->size());
    for (int i = 0; i < al->size(); ++i) {
      vec[i] = this->asBoolVar((*al)[i]);
    }
    return vec;
  }

  Nutmeg::Vector<Nutmeg::Int> NutmegSolverInstance::asInt(ArrayLit* al) {
    Nutmeg::Vector<Nutmeg::Int> vec(al->size());
    for (int i = 0; i < al->size(); ++i) {
      vec[i] = this->asInt((*al)[i]);
    }
    return vec;
  }

  Nutmeg::IntVar NutmegSolverInstance::asIntVar(Expression* e) {
    if (e->type().isvar()) {
      NutmegVariable& var = resolveVar(follow_id_to_decl(e));
      assert(var.isInt());
      return var.intVar();
    } else {
      IntVal i;
      if(auto il = e->dyn_cast<IntLit>()) {
        i = il->v().toInt();
      } else if(auto bl = e->dyn_cast<BoolLit>()) {
        i = bl->v();
      } else {
        std::stringstream ssm; ssm << "Expected bool or int literal instead of: " << *e;
        throw InternalError(ssm.str());
      }
      if (i == 0) {
        return _solver.get_zero();
      } else {
        const auto constant = static_cast<Nutmeg::Int>(i.toInt());
        return _solver.add_int_var(constant, constant, true, fmt::format("{}", constant));
      }
    }
  }

  Nutmeg::Vector<Nutmeg::IntVar> NutmegSolverInstance::asIntVar(ArrayLit* al) {
    Nutmeg::Vector<Nutmeg::IntVar> vec(al->size());
    for (int i = 0; i < al->size(); ++i) {
      vec[i] = this->asIntVar((*al)[i]);
    }
    return vec;
  }

  void NutmegSolverInstance::printStatistics(bool fLegend) {
      /*
    auto& st = _solver.data->stats;
    auto& out = getSolns2Out()->getOutput();

    out << "%%%mzn-stat: failures=" << st.conflicts << std::endl; // TODO: Statistic name
    out << "%%%mzn-stat: solveTime=" << st.time << std::endl;
    out << "%%%mzn-stat: solutions=" << st.solutions << std::endl;
    out << "%%%mzn-stat: restarts=" << st.restarts << std::endl;
    out << "%%%mzn-stat: nogoods=" << st.num_learnts << std::endl; // TODO: Statistic name
    out << "%%%mzn-stat: learntLiterals=" << st.num_learnt_lits << std::endl; // TODO: Statistic name
       */
  }

  Nutmeg_SolverFactory::Nutmeg_SolverFactory() {
    SolverConfig sc("org.minizinc.nutmeg", getVersion(nullptr));
    sc.name("Nutmeg");
    sc.mznlib("-Gnutmeg");
    sc.mznlibVersion(1);
    sc.supportsMzn(false);
    sc.description(getDescription(nullptr));
    sc.tags({"api","mip","cp","int","lcg",});
    sc.stdFlags({"-a", "-s", "-t"});
    sc.extraFlags({
    });
    SolverConfigs::registerBuiltinSolver(sc);
  };

  SolverInstanceBase::Options* Nutmeg_SolverFactory::createOptions() {
    return new NutmegOptions;
  }

  SolverInstanceBase* Nutmeg_SolverFactory::doCreateSI(Env& env, std::ostream& log, SolverInstanceBase::Options* opt) {
    return new NutmegSolverInstance(env, log, opt);
  }

  bool Nutmeg_SolverFactory::processOption(SolverInstanceBase::Options* opt, int &i, std::vector<std::string> &argv) {
    auto _opt = static_cast<NutmegOptions*>(opt);
    if (argv[i]=="-a" || argv[i]=="--all-solutions") {
      _opt->all_solutions = true;
    } else if (argv[i]=="--conflicts") {
      if (++i==argv.size()) return false;
      int nodes = atoi(argv[i].c_str());
      if(nodes >= 0)
        _opt->conflicts = nodes;
    } else if (argv[i]=="-f") {
      _opt->free_search = true;
    } else if (argv[i]=="-n") {
      if (++i==argv.size()) {
        return false;
      }
      int n = atoi(argv[i].c_str());
      if(n >= 0) {
        _opt->nr_solutions = n;
      }
    } else if (argv[i]=="--obj-probe") {
      if (++i==argv.size()) {
        return false;
      }
      int limit = atoi(argv[i].c_str());
      if(limit >= 0) {
        _opt->obj_probe_limit = limit;
      }
    } else if (argv[i]=="--solver-statistics" || argv[i]=="-s") {
      _opt->statistics = true;
    } else if (argv[i]=="--solver-time-limit" || argv[i]=="-t") {
      if (++i==argv.size()) return false;
      int time = atoi(argv[i].c_str());
      if(time >= 0)
        _opt->time = std::chrono::milliseconds(time);
    } else {
      return false;
    }
    return true;
  }

  void Nutmeg_SolverFactory::printHelp(std::ostream &os) {
  }
}

