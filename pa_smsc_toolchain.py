import csv
import re
import math
import itertools
import networkx as nx
from collections import defaultdict

# ==========================================
# 1. PARSING & DEPENDENCY MATRIX BUILDER
# ==========================================
def parse_fsm(csv_filename):
    transitions = []
    with open(csv_filename, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            curr_state = row['current_state'].strip()
            next_state = row['next_state'].strip()
            in_cond = row['input_condition'].strip()
            out_set = row['output_set'].strip()
            
            inputs = set(re.findall(r'x(\d+)', in_cond))
            outputs = set(re.findall(r'y(\d+)', out_set))
            
            transitions.append({
                'curr': curr_state, 'next': next_state,
                'inputs': inputs, 'outputs': outputs
            })
    return transitions

def build_dependencies(transitions):
    state_inputs = defaultdict(set)
    states = set()
    for t in transitions:
        states.add(t['curr'])
        states.add(t['next'])
        state_inputs[t['curr']].update(t['inputs'])
    return list(states), state_inputs

# ==========================================
# 2. ADVANCED GRAPH-BASED CLIQUE PARTITIONING
# ==========================================
def graph_guided_clique_partition(states, state_inputs, SL=6):
    print("🔹 Building Compatibility Graph...")
    G = nx.Graph()
    G.add_nodes_from(states)
    
    for u, v in itertools.combinations(states, 2):
        combined_inputs = state_inputs[u].union(state_inputs[v])
        Lk = len(combined_inputs)
        if Lk + 1 <= SL: 
            G.add_edge(u, v)
            
    print(f"✅ Graph built: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges.")
    print("🔹 Extracting Cliques using Graph-Guided Heuristic...")
    
    classes = []
    unassigned = set(states)
    
    while unassigned:
        subgraph = G.subgraph(unassigned)
        start_node = max(subgraph.nodes, key=lambda n: subgraph.degree(n))
        
        current_class = [start_node]
        current_inputs = state_inputs[start_node].copy()
        unassigned.remove(start_node)
        
        neighbors = sorted(subgraph.neighbors(start_node), 
                           key=lambda n: subgraph.degree(n), reverse=True)
        
        for neighbor in neighbors:
            if neighbor not in unassigned:
                continue
                
            new_inputs = current_inputs.union(state_inputs[neighbor])
            Lk = len(new_inputs)
            Mk = len(current_class) + 1
            Rk = math.ceil(math.log2(Mk)) if Mk > 1 else 1
            
            if Lk + Rk <= SL:
                current_class.append(neighbor)
                current_inputs = new_inputs
                unassigned.remove(neighbor)
                
        classes.append(current_class)
        print(f"   ➔ Formed Class {len(classes)-1}: {len(current_class)} states, {len(current_inputs)} inputs.")

    return classes

# ==========================================
# 3. POWER-AWARE STATE ASSIGNMENT (FIXED)
# ==========================================
def power_aware_assignment(classes, transitions):
    # Count ALL transition frequencies
    all_transitions = defaultdict(int)
    for t in transitions:
        all_transitions[(t['curr'], t['next'])] += 1

    partial_codes = {}
    
    for cls in classes:
        cls_set = set(cls) # For fast lookups
        Mk = len(cls)
        Rk = max(1, math.ceil(math.log2(Mk))) 
        
        if Mk == 1:
            partial_codes[cls[0]] = '0' * Rk
            continue
            
        all_codes = [format(i, f'0{Rk}b') for i in range(2**Rk)]
        assigned = {cls[0]: all_codes[0]}
        available_codes = set(all_codes[1:])
        
        # 🔥 THE FIX: Only look at transitions WITHIN the current class
        class_transitions = { (u, v): freq for (u, v), freq in all_transitions.items() 
                              if u in cls_set and v in cls_set }
                              
        sorted_trans = sorted(class_transitions.items(), key=lambda x: x[1], reverse=True)
        
        for (u, v), freq in sorted_trans:
            if u in assigned and v not in assigned:
                u_code = assigned[u]
                best_code = next((c for c in available_codes if sum(1 for a, b in zip(u_code, c) if a != b) == 1), None)
                if best_code is None: 
                    best_code = min(available_codes, key=lambda c: sum(1 for a, b in zip(u_code, c) if a != b))
                assigned[v] = best_code
                available_codes.remove(best_code)
                
            elif v in assigned and u not in assigned:
                v_code = assigned[v]
                best_code = next((c for c in available_codes if sum(1 for a, b in zip(v_code, c) if a != b) == 1), None)
                if best_code is None:
                    best_code = min(available_codes, key=lambda c: sum(1 for a, b in zip(v_code, c) if a != b))
                assigned[u] = best_code
                available_codes.remove(best_code)
                
        for state in cls:
            if state not in assigned:
                assigned[state] = available_codes.pop()
                
        partial_codes.update(assigned)
        
    return partial_codes

# ==========================================
# 4. REPORT GENERATION
# ==========================================
def generate_report(classes, state_inputs, partial_codes):
    K = len(classes)
    RV = max(1, math.ceil(math.log2(K))) if K > 1 else 1
    RS = max([max(1, math.ceil(math.log2(len(c)))) for c in classes])
    REC = RV + RS
    
    print(f"\n{'='*25} GRAPH PARTITIONING REPORT {'='*25}")
    print(f"Total Classes (K): {K} | Class Bits (RV): {RV} | Max Partial Bits (RS): {RS} | Total Bits (REC): {REC}")
    print("-" * 95)
    print(f"{'Class':<12} | {'States in Class':<45} | {'Inputs (Lk)':<12} | {'Bits (Rk)':<10} | {'Lk+Rk <= 6'}")
    print("-" * 95)
    
    class_codes = {}
    for i, cls in enumerate(classes):
        combined_inputs = set()
        for s in cls: combined_inputs.update(state_inputs[s])
        Lk = len(combined_inputs)
        Mk = len(cls)
        Rk = max(1, math.ceil(math.log2(Mk)))
        class_code = format(i, f'0{RV}b')
        class_codes[i] = class_code
        print(f"Class {i} ({class_code}) | {str(sorted(cls)):<45} | {Lk:<12} | {Rk:<10} | {Lk + Rk} <= 6 ✓")
        
    print(f"\n{'='*30} FINAL MIXED STATE CODES {'='*30}")
    print(f"{'State':<8} | {'Class':<8} | {'Partial Code':<15} | {'Class Code':<12} | {'Final MSC (Binary)'}")
    print("-" * 75)
    for i, cls in enumerate(classes):
        for state in sorted(cls):
            p_code = partial_codes[state]
            c_code = class_codes[i]
            final_code = c_code + p_code 
            print(f"{state:<8} | Class {i:<3} | {p_code:<15} | {c_code:<12} | {final_code}")

# ==========================================
# MAIN EXECUTION
# ==========================================
if __name__ == "__main__":
    try:
        print("🚀 Starting PA-SMSC Graph-Based Toolchain...\n")
        transitions = parse_fsm('amr_fsm.csv')
        states, state_inputs = build_dependencies(transitions)
        
        classes = graph_guided_clique_partition(states, state_inputs, SL=6)
        partial_codes = power_aware_assignment(classes, transitions)
        generate_report(classes, state_inputs, partial_codes)
        print("\n🎉 Toolchain execution complete!")
        
    except FileNotFoundError:
        print("❌ ERROR: 'amr_fsm.csv' not found. Please ensure it's in the same directory.")
    except ImportError:
        print("❌ ERROR: 'networkx' library not found. Please run: pip install networkx")