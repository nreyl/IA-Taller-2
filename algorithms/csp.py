from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from algorithms.problems_csp import DroneAssignmentCSP


def backtracking_search(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Basic backtracking search without optimizations.

    Tips:
    - An assignment is a dictionary mapping variables to values (e.g. {X1: Cell(1,2), X2: Cell(3,4)}).
    - Use csp.assign(var, value, assignment) to assign a value to a variable.
    - Use csp.unassign(var, assignment) to unassign a variable.
    - Use csp.is_consistent(var, value, assignment) to check if an assignment is consistent with the constraints.
    - Use csp.is_complete(assignment) to check if the assignment is complete (all variables assigned).
    - Use csp.get_unassigned_variables(assignment) to get a list of unassigned variables.
    - Use csp.domains[var] to get the list of possible values for a variable.
    - Use csp.get_neighbors(var) to get the list of variables that share a constraint with var.
    - Add logs to measure how good your implementation is (e.g. number of assignments, backtracks).

    You can find inspiration in the textbook's pseudocode:
    Artificial Intelligence: A Modern Approach (4th Edition) by Russell and Norvig, Chapter 5: Constraint Satisfaction Problems
    """
    conteo = {"asignaciones": 0, "backtracks": 0}

    def backtrack(assignment: dict[str, str]) -> dict[str, str] | None:
        if csp.is_complete(assignment):
            return assignment

        variables_sin_asignar = csp.get_unassigned_variables(assignment)
        var = variables_sin_asignar[0]

        for valor in csp.domains[var]:
            conteo["asignaciones"] += 1
            if csp.is_consistent(var, valor, assignment):
                csp.assign(var, valor, assignment)
                resultado = backtrack(assignment)
                if resultado is not None:
                    return resultado
                csp.unassign(var, assignment)
                conteo["backtracks"] += 1

        return None

    solucion = backtrack({})
    print(f"[backtracking] asignaciones intentadas: {conteo['asignaciones']}, backtracks: {conteo['backtracks']}")
    return solucion


def backtracking_fc(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Backtracking search with Forward Checking.

    Tips:
    - Forward checking: After assigning a value to a variable, eliminate inconsistent values from
      the domains of unassigned neighbors. If any neighbor's domain becomes empty, backtrack immediately.
    - Save domains before forward checking so you can restore them on backtrack.
    - Use csp.get_neighbors(var) to get variables that share constraints with var.
    - Use csp.is_consistent(neighbor, val, assignment) to check if a value is still consistent.
    - Forward checking reduces the search space by detecting failures earlier than basic backtracking.
    """
    conteo = {"asignaciones": 0, "backtracks": 0, "podas": 0}

    def backtrack(assignment: dict[str, str]) -> dict[str, str] | None:
        if csp.is_complete(assignment):
            return assignment

        variables_sin_asignar = csp.get_unassigned_variables(assignment)
        var = variables_sin_asignar[0]

        for valor in csp.domains[var]:
            conteo["asignaciones"] += 1
            if csp.is_consistent(var, valor, assignment):
                csp.assign(var, valor, assignment)

                # Guardar dominios antes de propagar
                dominios_guardados: dict[str, list[str]] = {}
                for vecino in csp.get_neighbors(var):
                    if vecino not in assignment:
                        dominios_guardados[vecino] = list(csp.domains[vecino])

                # Forward checking: eliminar valores inconsistentes de vecinos
                dominio_vacio = False
                for vecino in csp.get_neighbors(var):
                    if vecino in assignment:
                        continue
                    valores_a_eliminar = []
                    for val_vecino in csp.domains[vecino]:
                        if not csp.is_consistent(vecino, val_vecino, assignment):
                            valores_a_eliminar.append(val_vecino)
                            conteo["podas"] += 1
                    for v in valores_a_eliminar:
                        csp.domains[vecino].remove(v)
                    if len(csp.domains[vecino]) == 0:
                        dominio_vacio = True
                        break

                if not dominio_vacio:
                    resultado = backtrack(assignment)
                    if resultado is not None:
                        # Restaurar dominios antes de retornar
                        for vecino, dominio in dominios_guardados.items():
                            csp.domains[vecino] = dominio
                        return resultado

                # Restaurar dominios al hacer backtrack
                for vecino, dominio in dominios_guardados.items():
                    csp.domains[vecino] = dominio

                csp.unassign(var, assignment)
                conteo["backtracks"] += 1

        return None

    solucion = backtrack({})
    print(f"[backtracking_fc] asignaciones intentadas: {conteo['asignaciones']}, backtracks: {conteo['backtracks']}, podas: {conteo['podas']}")
    return solucion


def backtracking_ac3(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Backtracking search with AC-3 arc consistency.

    Tips:
    - AC-3 enforces arc consistency: for every pair of constrained variables (Xi, Xj), every value
      in Xi's domain must have at least one supporting value in Xj's domain.
    - Run AC-3 before starting backtracking to reduce domains globally.
    - After each assignment, run AC-3 on arcs involving the assigned variable's neighbors.
    - If AC-3 empties any domain, the current assignment is inconsistent - backtrack.
    - You can create helper functions such as:
      - a values_compatible function to check if two variable-value pairs are consistent with the constraints.
      - a revise function that removes unsupported values from one variable's domain.
      - an ac3 function that manages the queue of arcs to check and calls revise.
      - a backtrack function that integrates AC-3 into the search process.
    """

        
    def backtrack(assignment: dict[str, str]) -> dict[str, str] | None:
        
        if csp.is_complete(assignment):
            return assignment

        variables_sin_asignar = csp.get_unassigned_variables(assignment)
        var = variables_sin_asignar[0]

        for valor in csp.domains[var][:]:   
            
            conteo["asignaciones"] += 1
            
            if csp.is_consistent(var, valor, assignment):
                
                csp.assign(var, valor, assignment)

                removals_log = []
                
                queue = []
                for vecino in csp.get_neighbors(var):
                    if vecino not in assignment:
                        queue.append((vecino, var))


                consistente = ac3(queue, assignment, removals_log)

                if consistente:
                    resultado = backtrack(assignment)
                    if resultado is not None:
                        return resultado

                while removals_log:
                    v, val = removals_log.pop()
                    if val not in csp.domains[v]:
                         csp.domains[v].append(val)
                         
                csp.unassign(var, assignment)
                conteo["backtracks"] += 1

        return None


    def values_compatible(Xi, x, Xj, y, assignment):
        
        was_assigned_Xi = Xi in assignment
        was_assigned_Xj = Xj in assignment
        
        old_Xi = assignment.get(Xi)
        old_Xj = assignment.get(Xj)
        
        csp.assign(Xi, x, assignment)
        csp.assign(Xj, y, assignment)
        
        compatible = (
            csp.is_consistent(Xi, x, assignment)
            and
            csp.is_consistent(Xj, y, assignment)
        )
        
        if was_assigned_Xi:
            assignment[Xi] = old_Xi
        else:
            del assignment[Xi]
        
        if was_assigned_Xj:
            assignment[Xj] = old_Xj
        else:
            del assignment[Xj]
        
        return compatible


    def revise(Xi, Xj, assignment, removals_log:list):
        
        revised = False
        
        for x in csp.domains[Xi][:]:
            
            supported = False
            
            for y in csp.domains[Xj]:
                
                if values_compatible(Xi, x, Xj, y, assignment):
                    supported = True
                    break
            
            if not supported:
                csp.domains[Xi].remove(x)
                removals_log.append((Xi, x))
                revised = True
        
        return revised
    
    def ac3(queue:list, assignment, removals):

        while queue:

            Xi, Xj = queue.pop(0)

            if revise(Xi, Xj, assignment, removals):

                conteo["revisions"]+=1
                
                if len(csp.domains[Xi]) == 0:
                    return False

                for i in csp.get_neighbors(Xi):
                    if i != Xj:
                        queue.append((i, Xi))

        return True
    
        
    conteo = {
    "asignaciones": 0,
    "backtracks": 0,
    "revisions": 0
    }
    
    
    queue = []
    for Xi in csp.domains:
        for Xj in csp.get_neighbors(Xi):
            queue.append((Xi,Xj))
    if not ac3(queue,{}, []):
        return None
    solucion = backtrack({})
    print(f"[backtracking_AC3] asignaciones intentadas: {conteo['asignaciones']}, backtracks: {conteo['backtracks']}, revisions: {conteo['revisions']}")
    return solucion


    
    




def backtracking_mrv_lcv(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Backtracking with Forward Checking + MRV + LCV.

    Tips:
    - Combine the techniques from backtracking_fc, mrv_heuristic, and lcv_heuristic.
    - MRV (Minimum Remaining Values): Select the unassigned variable with the fewest legal values.
      Tie-break by degree: prefer the variable with the most unassigned neighbors.
    - LCV (Least Constraining Value): When ordering values for a variable, prefer
      values that rule out the fewest choices for neighboring variables.
    - Use csp.get_num_conflicts(var, value, assignment) to count how many values would be ruled out for neighbors if var=value is assigned.
    """
    # TODO: Implement your code here (BONUS)
    return None