/** <module> Utilities for handling units of measure and the conversion between different units

  Copyright (C) 2011 Moritz Tenorth
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Moritz Tenorth
@license BSD

*/

:- module(knowrob_math,
    [
      quaternion_multiply/3,
      parse_vector/2,
      atomic_term/1,
      print_bracket_term/2,
      print_math_term/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).

:- rdf_meta
         atomic_term(r),
         print_bracket_term(r,?),
         print_math_term(r,?).

:- owl_parse('package://knowrob_common/owl/knowrob_math.owl').

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).


atomic_term(Term) :-
  nonvar(Term),
  rdf_has(Term, rdf:type, knowrob:'DoubleConstExpression');
  rdf_has(Term, rdf:type, knowrob:'DoubleVariableExpression').

print_math_term(Term, String) :-
  nonvar(Term),
  rdf_has(Term, knowrob:value, Value),
  strip_literal_type(Value, StrippedValue),
  atom_string(StrippedValue, String).

print_math_term(Term, String) :-
  nonvar(Term),
  rdf_has(Term, rdf:type, knowrob:'DoubleVariableExpression'),
  atom_string("?", String).

print_math_term(Term, String) :-
  nonvar(Term),
  rdf_has(Term, rdf:type, knowrob:'DoubleAdditionExpression'),
  findall(Summand, (rdf_has(Term, knowrob:summand, Summand)), Summands),
  Summands = [S1, S2],
  print_math_term(S1, String1),
  print_math_term(S2, String2),
  atom_string(" + ", MidString),
  string_concat(String1, MidString, TempString),
  string_concat(TempString, String2, String).

print_math_term(Term, String) :-
  nonvar(Term),
  rdf_has(Term, rdf:type, knowrob:'DoubleMultiplicationExpression'),
  findall(Factor, (rdf_has(Term, knowrob:factor, Factor)), Factors),
  Factors = [F1, F2],
  (atomic_term(F1) -> print_math_term(F1, String1) ; print_bracket_term(F1, String1)),
  (atomic_term(F2) -> print_math_term(F2, String2) ; print_bracket_term(F2, String2)),
  atom_string(" * ", MidString),
  string_concat(String1, MidString, TempString),
  string_concat(TempString, String2, String).

print_bracket_term(Term, String) :-
  print_math_term(Term, MathString),
  atom_string('(', OpenString),
  atom_string(')', ClosingString),
  string_concat(OpenString, MathString, TempString),
  string_concat(TempString, ClosingString, String).

% TODO: Write concatenate list of strings into string predicate

parse_vector([X|Y], [X|Y]).
parse_vector(In, Numbers) :-
  parse_vector(In, Numbers, ' ').
parse_vector(In, Numbers, Delimiter) :-
  atom(In),
  normalize_space(atom(In_Normalized),In),
  atomic_list_concat(Atoms, Delimiter, In_Normalized),
  findall(Num, (
    member(Atom,Atoms),
    atom_number(Atom,Num)
  ), Numbers),
  length(Atoms,L),
  length(Numbers,L).
  %jpl_call('org.knowrob.utils.MathUtil', 'parseVector', [In, ' '], OutArr),
  %not(OutArr = @(null)),
  %jpl_array_to_list(OutArr, Out).

quaternion_multiply(Q0,Q1,Multiplied) :-
  jpl_list_to_array(Q0, Q0_array),
  jpl_list_to_array(Q1, Q1_array),
  jpl_call('org.knowrob.utils.MathUtil', 'quaternionMultiply', [Q0_array, Q1_array], Out_array),
  jpl_array_to_list(Out_array, Multiplied).
