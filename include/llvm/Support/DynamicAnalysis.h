//=-------------------- llvm/Support/DynamicAnalysis.h ------======= -*- C++ -*//
//
//                     The LLVM Compiler Infrastructure
//
//  Victoria Caparros Cabezas <caparrov@inf.ethz.ch>
//===----------------------------------------------------------------------===//


#ifndef LLVM_SUPPORT_DYNAMIC_ANALYSIS_H
#define LLVM_SUPPORT_DYNAMIC_ANALYSIS_H

//#define INTERPRETER
//#define EFF_TBV

#include "../../../lib/ExecutionEngine/Interpreter/Interpreter.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/Debug.h"
#include "llvm/DebugInfo.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/Support/InstIterator.h"
#include "llvm/Support/GetElementPtrTypeIterator.h"



#ifdef INTERPRETER
#include "llvm/Support/top-down-size-splay.hpp"
//#include "llvm/Support/ValuesAnalysis.h"
#else
#include "top-down-size-splay.hpp"
//#include "ValuesAnalysis.h"
#endif

//#define INT_FP_OPS

//#define  SOURCE_CODE_ANALYSIS

#include <iostream>
#include <map>
#include <stdarg.h>
#include <stdio.h>
#include <fstream>
#include <string>
#include <list>
#include <vector>
#include <cstdlib>



//#include <boost/unordered_map.hpp>
#include <map>


#ifdef SOURCE_CODE_ANALYSIS
#include <unordered_map>
#endif
#include <deque>
#define ROUND_REUSE_DISTANCE
#define NORMAL_REUSE_DISTRIBUTION


//#define DEBUG_SOURCE_CODE_LINE_ANALYSIS
//#define PRINT_DEPENDENCIES
#define DEBUG_MEMORY_TRACES
#define DEBUG_REUSE_DISTANCE
#define DEBUG_GENERIC
#define DEBUG_REGISTER_FILE
//#define DEBUG_DEPS_FUNCTION_CALL
//#define DEBUG_SPAN_CALCULATION
//#define DEBUG_AGU
//#define DEBUG_OOO_BUFFERS
#define DEBUG_ISSUE_CYCLE
#define DEBUG_POINTERS_TO_MEMORY
//#define DEBUG_PHI_NODE
//#define DEBUG_FUNCTION_CALL_STACK
//#define DEBUG_PREFETCHER

//#define ILP_DISTRIBUTION
//#define MICROSCHEDULING

//#define ASSERT


#define STORES_IN_REGISTER
#define MOO_BUFFERS

#define INTERMEDIATE_RESULTS_STACK


/*The YMM registers are aliased over the older 128-bit XMM registers used for
 Intel SSE, treating the XMM registers as the lower half of the corresponding
 YMM register.
 */

#define INF -1



// ====== Instructions considered in the analysis ===========================

#define INT_ADD          -1
#define INT_SUB          -1
#define INT_MUL          -1
#define INT_DIV          -1
#define INT_REM         -1

#define INT_LD_4_BITS    -1
#define INT_LD_8_BITS    -1
#define INT_LD_16_BITS   -1
#define INT_LD_32_BITS   -1
#define INT_LD_64_BITS   -1
#define INT_LD_80_BITS   -1
#define INT_LD_128_BITS  -1
#define INT_ST_4_BITS    -1
#define INT_ST_8_BITS    -1
#define INT_ST_16_BITS   -1
#define INT_ST_32_BITS   -1
#define INT_ST_64_BITS   -1
#define INT_ST_80_BITS   -1
#define INT_ST_128_BITS  -1


#define FP32_ADD           0
#define FP64_ADD           0
#define FP32_SUB           0
#define FP64_SUB           0
#define FP32_MUL           0
#define FP64_MUL           0
#define FP32_FMA           0
#define FP64_FMA           0
#define FP32_DIV           0
#define FP64_DIV           0
#define FP_REM          -1
#define FP_LD_16_BITS    1
#define FP_LD_32_BITS    1
#define FP_LD_64_BITS    1
#define FP_LD_80_BITS    1
#define FP_LD_128_BITS   1
#define FP_ST_16_BITS    1
#define FP_ST_32_BITS    1
#define FP_ST_64_BITS    1
#define FP_ST_80_BITS    1
#define FP_ST_128_BITS   1
#define MISC_MEM        -1
#define CTRL            -1
#define FP32_SHUFFLE       0
#define FP64_SHUFFLE       0
#define FP64_MOV       	   0
#define FP32_MOV           0
#define FP32_BLEND         0
#define FP64_BLEND         0

#define MISC            -1





// =================== ARM Cortex A9 config ================================//

/*

#define ARM_CORTEX_A9_EXECUTION_UNITS 6
#define ARM_CORTEX_DISPATCH_PORTS 2
#define ARM_CORTEX_BUFFERS 5
#define ARM_CORTEX_AGUS 0
#define ARM_CORTEX_LOAD_AGUS 0
#define ARM_CORTEX_STORE_AGUS 0


#define ARM_CORTEX_N_COMP_EXECUTION_UNITS 4
#define ARM_CORTEX_N_MEM_EXECUTION_UNITS 5
#define ARM_CORTEX_N_AGUS 1

#define ARM_CORTEX_FP_ADDER  0
#define ARM_CORTEX_FP_MULTIPLIER 1
#define ARM_CORTEX_FP_DIVIDER  2
#define ARM_CORTEX_FP_SHUFFLE  3
#define ARM_CORTEX_L1_LOAD_CHANNEL 4
#define ARM_CORTEX_L1_STORE_CHANNEL  5
#define ARM_CORTEX_L2_LOAD_CHANNEL 6
#define ARM_CORTEX_L2_STORE_CHANNEL  7
#define ARM_CORTEXM_L3_LOAD_CHANNEL 8
#define ARM_CORTEX_L3_STORE_CHANNEL  9
#define ARM_CORTEX_MEM_LOAD_CHANNEL 10
#define ARM_CORTEX_MEM_STORE_CHANNEL  11
#define ARM_CORTEX_ADDRESS_GENERATION_UNIT 12
#define ARM_CORTEX_STORE_ADDRESS_GENERATION_UNIT 13
#define ARM_CORTEX_LOAD_ADDRESS_GENERATION_UNIT 14

*/


// =================== Atom config ================================//
/*


#define ATOM_EXECUTION_UNITS 6
#define ATOM_DISPATCH_PORTS 2
#define ATOM_BUFFERS 5
#define ATOM_AGUS 0
#define ATOM_LOAD_AGUS 0
#define ATOM_STORE_AGUS 0


#define ATOM_N_COMP_EXECUTION_UNITS 4
#define ATOM_N_MEM_EXECUTION_UNITS 5
#define ATOM_N_AGUS 2

#define ATOM_FP_ADDER  0
#define ATOM_FP_MULTIPLIER 1
#define ATOM_FP_DIVIDER  2
#define ATOM_FP_SHUFFLE  3
#define ATOM_L1_LOAD_CHANNEL 4
#define ATOM_L1_STORE_CHANNEL  5
#define ATOM_L2_LOAD_CHANNEL 6
#define ATOM_L2_STORE_CHANNEL  7
#define ATOM_L3_LOAD_CHANNEL 8
#define ATOM_L3_STORE_CHANNEL  9
#define ATOM_MEM_LOAD_CHANNEL 10
#define ATOM_MEM_STORE_CHANNEL  11
#define ATOM_ADDRESS_GENERATION_UNIT 12
#define ATOM_STORE_ADDRESS_GENERATION_UNIT 13
#define ATOM_LOAD_ADDRESS_GENERATION_UNIT 14
*/



/*
// =================== Generic config ================================//



#define GENERIC_AGUS 2
#define GENERIC_LOAD_AGUS 0
#define GENERIC_STORE_AGUS 1


// Define execution units
#define GENERIC_N_MEM_EXECUTION_UNITS 5
#define GENERIC_N_COMP_EXECUTION_UNITS 4
#define GENERIC_N_AGUS 2

#define GENERIC_FP_ADDER  0
#define GENERIC_FP_MULTIPLIER 1
#define GENERIC_FP_DIVIDER  2
#define GENERIC_FP_SHUFFLE  3
#define GENERIC_L1_LOAD_CHANNEL 4
#define GENERIC_L1_STORE_CHANNEL  5
#define GENERIC_L2_LOAD_CHANNEL 6
#define GENERIC_L2_STORE_CHANNEL  7
#define GENERIC_L3_LOAD_CHANNEL 8
#define GENERIC_L3_STORE_CHANNEL  9
#define GENERIC_MEM_LOAD_CHANNEL 10
#define GENERIC_MEM_STORE_CHANNEL  11
#define GENERIC_ADDRESS_GENERATION_UNIT 12
#define GENERIC_STORE_ADDRESS_GENERATION_UNIT 13
#define GENERIC_LOAD_ADDRESS_GENERATION_UNIT 14


*/

//===================== INSTRUCTION TYPES ========================//


enum {
  
  // Arithmetic computation nodes
  FP32_ADD_NODE = 0,
  FP64_ADD_NODE,

  FP32_MUL_NODE,
  FP64_MUL_NODE,

  FP32_FMA_NODE,
  FP64_FMA_NODE,

  FP32_DIV_NODE,
  FP64_DIV_NODE,

  
  
// Vector nodes  
  FP32_SHUFFLE_NODE,
  FP64_SHUFFLE_NODE,

  FP32_BLEND_NODE,
  FP64_BLEND_NODE,

  FP32_MOV_NODE,
  FP64_MOV_NODE,
  
//Memory nodes
  REGISTER_LOAD_NODE,
  REGISTER_STORE_NODE,
  L1_LOAD_NODE,
  L1_STORE_NODE,
  L2_LOAD_NODE,
   FIRST_PREFETCH_LEVEL=L2_LOAD_NODE,
  L2_STORE_NODE,
  L3_LOAD_NODE,
  L3_STORE_NODE,
  MEM_LOAD_NODE,
  MEM_STORE_NODE,
 
  RS_STALL_NODE,
  ROB_STALL_NODE,
  LB_STALL_NODE,
  SB_STALL_NODE,
  LFB_STALL_NODE,
 
  AGU_NODE,
  STORE_AGU_NODE  = AGU_NODE,
  LOAD_AGU_NODE = AGU_NODE,
  PORT_0_NODE,
  PORT_1_NODE,
  PORT_2_NODE,
  PORT_3_NODE,
  PORT_4_NODE,
  PORT_5_NODE,
 
  
  L2_LOAD_PREFETCH_NODE,
  L2_STORE_PREFETCH_NODE,
  L3_LOAD_PREFETCH_NODE,
  L3_STORE_PREFETCH_NODE,
  MEM_LOAD_PREFETCH_NODE,
  MEM_STORE_PREFETCH_NODE,
 
  
  TOTAL_NODES
};


//===================== EXECUTION UNITS ========================//
// Execution units are all those resources for which we have a cycle in
// FullOccupacyTree


enum {
  FP32_ADDER = 0,
  FP64_ADDER,

  FP32_MULTIPLIER,
  FP64_MULTIPLIER,

  FP32_FMADDER,
  FP64_FMADDER,

  FP32_DIVIDER,
  FP64_DIVIDER,

  FP32_SHUFFLE_UNIT,
  FP64_SHUFFLE_UNIT,

  FP32_BLEND_UNIT,
  FP64_BLEND_UNIT,

  FP32_MOV_UNIT,
  FP64_MOV_UNIT,

  
  REGISTER_LOAD_CHANNEL,
  REGISTER_STORE_CHANNEL = REGISTER_LOAD_CHANNEL,
  L1_LOAD_CHANNEL,
  L1_STORE_CHANNEL,
  L2_LOAD_CHANNEL,
  L2_STORE_CHANNEL = L2_LOAD_CHANNEL,
  L3_LOAD_CHANNEL,
  L3_STORE_CHANNEL = L3_LOAD_CHANNEL,
  MEM_LOAD_CHANNEL,
  MEM_STORE_CHANNEL = MEM_LOAD_CHANNEL,

  RS_STALL,
  ROB_STALL,
  LB_STALL,
  SB_STALL,
  LFB_STALL,
  ADDRESS_GENERATION_UNIT,
  STORE_ADDRESS_GENERATION_UNIT = ADDRESS_GENERATION_UNIT,
  LOAD_ADDRESS_GENERATION_UNIT = ADDRESS_GENERATION_UNIT,
  PORT_0,
  PORT_1,
  PORT_2,
  PORT_3,
  PORT_4,
  PORT_5,
 /* RS_STALL,
  ROB_STALL,
  LB_STALL,
  SB_STALL,
  LFB_STALL,*/
  MAX_RESOURCE_VALUE
};



// ====================================================================//





// =================== Sandy Bridge config ================================//


// These are execution units, o resources -> there is an available and
// and full occupancy tree from them
/*

#define SANDY_BRIDGE_EXECUTION_UNITS MEM_STORE_CHANNEL+1
#define SANDY_BRIDGE_NODES TOTAL_NODES+1

#define SANDY_BRIDGE_ARITHMETIC_NODES 3 // ADD (SUB), MUL, DIV
#define SANDY_BRIDGE_ARITHMETIC_EXECUTION_UNITS 3

#define SANDY_BRIDGE_MOV_NODES 3 
#define SANDY_BRIDGE_MOV_EXECUTION_UNITS 3


#define SANDY_BRIDGE_MEM_NODES 10 // Before 8, we add 1 for register file size
#define SANDY_BRIDGE_MEM_EXECUTION_UNITS 6  // Before 5


#define SANDY_BRIDGE_DISPATCH_PORTS 6
#define SANDY_BRIDGE_BUFFERS 5
#define SANDY_BRIDGE_AGUS 2
#define SANDY_BRIDGE_LOAD_AGUS 0
#define SANDY_BRIDGE_STORE_AGUS 0
#define SANDY_BRIDGE_PREFETCH_NODES 3



*/



#define EXECUTION_UNITS MEM_STORE_CHANNEL+1
#define NODES TOTAL_NODES+1

#define ARITHMETIC_NODES 8 // ( ADD (SUB), MUL, FMA, DIV )*2 -> SP and DP
#define ARITHMETIC_EXECUTION_UNITS 8

#define MOV_NODES 6 // * 2
#define MOV_EXECUTION_UNITS 6 // * 2, to consider single and double precision as separate nodes


#define MEM_NODES 10 // Before 8, we add 1 for register file size
#define MEM_EXECUTION_UNITS 6  // Before 5


#define DISPATCH_PORTS 6
#define BUFFERS 5
#define AGUS 2
#define LOAD_AGUS 0
#define STORE_AGUS 0
#define PREFETCH_NODES 3







#define N_TOTAL_NODES N_COMP_NODES+N_MEM_NODES+N_BUFFER_NODES+N_PREFETCH_RESOURCES+N_MISC_RESOURCES
#define N_MEM_RESOURCES_START L1_LOAD_CHANNEL
#define N_MEM_RESOURCES_END MEM_STORE_CHANNEL

//Define Microarchitectures
#define SANDY_BRIDGE 0
#define ATOM 1



using namespace llvm;
using namespace std;
using namespace SplayTree;
using namespace SplayTreeBoolean;
using namespace SimpleSplayTree;
using namespace ComplexSplayTree;

// For FullOccupancyCyles, the vector has a different meaning that for AvailableCycles.
// Each element of the vector contains the elements of the tree in a corresponding
// rage.
static const unsigned SplitTreeRange = 131072;
//static const unsigned SplitTreeRange = 262144;
//static const int SplitTreeRange = 32768;



/*
bool operator< ( PointerToMemory a, PointerToMemory b );
bool operator== ( PointerToMemory a, PointerToMemory b );
*/
// LessThanComparable: necessary for implementing a map with a PointerToMemory struct as a key


/*
bool operator==(const PointerToMem &lhs, const PointerToMem &rhs)
{
  return lhs.BasePointer == rhs.BasePointer && lhs.ArrayOffset == rhs.ArrayOffset && lhs.WithinArrayOffset == rhs.WithinArrayOffset ;
}

std::size_t hash_value(const PointerToMem &a)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, a.BasePointer);
  boost::hash_combine(seed, a.ArrayOffset);
  boost::hash_combine(seed, a.WithinArrayOffset);
  return seed;
}

*/



// ==================================================================================================
//  Data structures definitions for tracking pointer to memory
//===================================================================================================

struct PointerToMemory{
	Value * BasePointer;
	Value * Offset1;
	Value * Offset2;
	Value * Offset3;
};
struct PointerToMemoryInstance{
	int64_t PTMindex;
	int64_t IterationCount;
};

struct InstructionValue{
	Value * v;
	unsigned valueRep;
};


struct InstructionValueInstance{
//	/InstructionValue IV;
	Value * v;
	unsigned valueRep;
	int64_t valueInstance;
};



struct InstructionValueInstanceInfo{
	PointerToMemoryInstance PTMI;
	uint64_t Address;
	unsigned nUses;
};

bool operator <(const PointerToMemory& x, const PointerToMemory& y);

bool operator <(const InstructionValueInstance& x, const InstructionValueInstance& y);



// ==================================================================================================
//  End of data structures definitions for tracking pointer to memory
//===================================================================================================


struct CacheLineInfo{
  uint64_t IssueCycle;
  uint64_t LastAccess;
  // uint64_t ReuseDistance;
};


struct InstructionDispatchInfo{
  uint64_t IssueCycle;
  uint64_t CompletionCycle;
};


struct LessThanOrEqualValuePred
{
  uint64_t CompareValue;
  
  bool operator()(const uint64_t Value) const
  {
    return Value <= CompareValue;
  }
};

struct StructMemberLessThanOrEqualThanValuePred
{
  const uint64_t CompareValue;
  
  bool operator()(const InstructionDispatchInfo& v) const
  {
    return v.CompletionCycle <= CompareValue;
  }
};


  class TBV_node {
  public:
#ifdef EFF_TBV
    TBV_node():BitVector(SplitTreeRange) {
    }
bool e;
#else
TBV_node():BitVector(MAX_RESOURCE_VALUE) {
    }
#endif
  //  vector<bool> BitVector;
  dynamic_bitset<> BitVector; // from boost
  bool get_node(uint64_t bitPosition);
   void insert_node(uint64_t bitPosition);
bool get_node_nb(uint64_t bitPosition);
bool empty();
#ifdef SOURCE_CODE_ANALYSIS
//    set<uint64_t> SourceCodeLines;
    vector<pair<unsigned,unsigned>> SourceCodeLinesOperationPair;
#endif
    

  };


class TBV {

  
private:
  vector<TBV_node> tbv_map;
  bool e;
  
public:
  TBV();
  void insert_source_code_line(uint64_t key, unsigned SourceCodeLine, unsigned Resource);
vector<pair<unsigned,unsigned> >  get_source_code_lines(uint64_t key);
bool get_size();
void resize();

  bool get_node(uint64_t key, unsigned bitPosition);
   void insert_node(uint64_t key, unsigned bitPosition);

  bool get_node_nb(uint64_t key, unsigned bitPosition);

  void delete_node(uint64_t key, unsigned bitPosition);
  bool empty();
};




struct ACTNode {
public:
  uint64_t key;
  int32_t issueOccupancy;
  int32_t widthOccupancy;
  int32_t occupancyPrefetch;
  uint64_t address;
};

class ACT {
private:
  vector< TBV> act_vec;
  
public:
  bool get_node_ACT(uint64_t, unsigned);
  void push_back(ACTNode*, unsigned);
  void DebugACT();
  size_t size();
  void clear();
};



uint64_t BitScan(vector< TBV> &FullOccupancyCyclesTree, uint64_t key, unsigned bitPosition);

class DynamicAnalysis {
  
public:
  
  unsigned TotalResources;
  unsigned nExecutionUnits;
  unsigned nArithmeticExecutionUnits;
  unsigned nMovExecutionUnits;
  unsigned nMemExecutionUnits;
  unsigned nPorts;
  unsigned nBuffers;
  unsigned nAGUs;
  unsigned nLoadAGUs;
  unsigned nStoreAGUs;
  unsigned nNodes;
  unsigned nPrefetchNodes;
  unsigned nArithmeticNodes;
  unsigned nMovNodes;
  unsigned nMemNodes;
  unsigned MemoryWordSize;
  unsigned CacheLineSize;
  

  // Register file size
  unsigned RegisterFileSize;
  //Cache sizes are specified in number of cache lines of size CacheLineSize
  unsigned L1CacheSize;
  unsigned L2CacheSize;
  unsigned LLCCacheSize;
  
  unsigned BitsPerCacheLine;
  
  unsigned VectorWidth;
  
  //TODO: COmment out
  vector<uint64_t> FlopLatencies;
  vector<uint64_t> MemLatencies;
  unsigned MaxLatencyResources;
  
  //For every node, the execution unit in which it executes.
  vector<unsigned> ExecutionUnit;

  vector<vector<unsigned> > DispatchPort;
  
  vector<unsigned> ExecutionUnitsLatency;
  vector<double> ExecutionUnitsThroughput;
  vector<int> ExecutionUnitsParallelIssue;
  vector<unsigned> PortsWidth;
  
  vector<unsigned> IssueCycleGranularities;
  vector<unsigned> AccessWidths;
  
  vector<float> Throughputs;
  vector<float> PortsWidths;
  // Port-Node type that can be executed on that port.
  vector<vector<unsigned> > PortsBindings;
  
  vector<bool> ShareThroughputAmongPorts;
  
  
  // vector<float> FlopThroughputs;
  // vector<float> MemThroughputs;
  
  // TODO: COmment out
  vector<float> IssueThroughputs;
  vector<float> IssueWidths;
  
  
  vector<unsigned > AccessGranularities;
  vector<string> ResourcesNames;
  vector<string> NodesNames;
  
  string OutputDir;
  
  unsigned NRegisterSpillsLoads;
  unsigned NRegisterSpillsStores;
  
  bool LimitILP;
  bool LimitMLP;
  
  uint64_t BasicBlockBarrier;
  uint64_t BasicBlockLookAhead;
  int64_t RemainingInstructionsFetch;
  uint64_t InstructionFetchCycle;
  uint64_t LoadDispatchCycle;
  uint64_t StoreDispatchCycle;
  vector<uint64_t> ReservationStationIssueCycles;
  deque<uint64_t> ReorderBufferCompletionCycles;
  vector<uint64_t> LoadBufferCompletionCycles;
  SimpleTree<uint64_t> *LoadBufferCompletionCyclesTree;
  vector<uint64_t> StoreBufferCompletionCycles;
  vector<uint64_t> LineFillBufferCompletionCycles;
  vector<InstructionDispatchInfo> DispatchToLoadBufferQueue;
  ComplexTree<uint64_t> *DispatchToLoadBufferQueueTree;
vector<pair<uint64_t,uint64_t> > DispatchToLoadBufferQueueTreeCyclesToRemove;

	
	bool SmallBuffers;
  
  vector<InstructionDispatchInfo> DispatchToStoreBufferQueue;
  vector<InstructionDispatchInfo> DispatchToLineFillBufferQueue;
  
  bool RARDependences;
  bool WarmCache;
  bool x86MemoryModel;
  bool SpatialPrefetcher;
  bool ConstraintPorts;
  bool ConstraintPortsx86;
  bool BlockPorts;
  bool ConstraintAGUs;
  bool FloatPrecision;
  unsigned PrefetchLevel;
  unsigned PrefetchDispatch;
  unsigned PrefetchTarget;
  unsigned PrefetchDestination;
  
  bool InOrderExecution;
  bool ReportOnlyPerformance;
  
  unsigned ReservationStationSize;
  int AddressGenerationUnits;
  int InstructionFetchBandwidth;
  unsigned ReorderBufferSize;
  unsigned LoadBufferSize;
  unsigned StoreBufferSize;
  unsigned LineFillBufferSize;
  
  string TargetFunction;
  uint8_t FunctionCallStack;

  bool VectorCode;
  bool SinglePrecisionCode;
  bool DoublePrecisionCode;
  
 // uint8_t uarch;
  
  unsigned SourceCodeLine;
  
  int rep;
  uint64_t globalAddrForArtificialMemOps;
  
#ifdef INT_FP_OPS
  // Begin new
  // Number of bytes transferred from/to L1, L2, LLC, Mem, Total respectively
  // Depends on program
  vector<unsigned> Q;
  
  // Throughput (bytes/second) for L1, L2, LLC, Mem respectively
  vector<float> Beta;
  
  // Computation throughput bound (ops/second)
  float Pi;
  // End new
#endif
  
  // Variables to track instructions count
  uint64_t TotalInstructions;
  uint64_t TotalSpan;
  uint64_t NFirstAccesses;
  vector<uint64_t> InstructionsCount;
  vector<uint64_t> InstructionsCountExtended;
  vector<uint64_t> ScalarInstructionsCountExtended;
  vector<uint64_t> VectorInstructionsCountExtended;
  vector<bool> InstructionsScalarVectorMixed;
  vector<uint64_t> InstructionsSpan;
  vector<uint64_t> InstructionsLastIssueCycle;
  vector<uint64_t> IssueSpan;
  vector<uint64_t> LatencyOnlySpan;
  vector<uint64_t> SpanGaps;
  vector<uint64_t> FirstNonEmptyLevel;
  vector<uint64_t> BuffersOccupancy;
  vector<uint64_t> LastIssueCycleVector;
  vector<double> AverageOverlapsCycles;
  vector<uint64_t> OverlapsCount;
  vector<double> AverageOverlaps;
  vector<double> OverlapsDerivatives;
  vector<double> OverlapsMetrics;

  float BuffersOccupancyThreshold;
  uint64_t LastIssueCycleFinal;
  
  vector<unsigned> MaxOccupancy;
  vector<bool> FirstIssue;
  
  deque<uint64_t> RegisterFile;
  
  uint64_t LastLoadIssueCycle;
  uint64_t LastStoreIssueCycle;
  
  
  vector< vector< unsigned > > DAGLevelsOccupancy;
  /*
   vector< Tree<uint64_t> * > AvailableCyclesTree;
   vector< Tree<uint64_t> * > FullOccupancyCyclesTree;
   
   vector< TreeBitVector<uint64_t> * > AvailableCyclesTreeBitVector;
   vector< TreeBitVector<uint64_t> * > FullOccupancyCyclesTreeTreeBitVector;
   */
  
  
  vector< Tree<uint64_t> * > AvailableCyclesTree;
  

 #ifdef EFF_TBV
 vector< TBV_node> FullOccupancyCyclesTree;
#else
 vector< TBV> FullOccupancyCyclesTree;
  #endif
  vector <Tree<uint64_t> * > StallCycles;
  vector <uint64_t> NInstructionsStalled;
  
  uint64_t MinLoadBuffer;
  uint64_t MaxDispatchToLoadBufferQueueTree;
  vector<ComplexTree<uint64_t> *> PointersToRemove;
  

  //---------------- CONTECH: NEW FINAL VERSIONS ----------------------
   vector< dynamic_bitset<> > CGSFCache;
   vector< dynamic_bitset<> > CISFCache;
   vector< dynamic_bitset<> > CLSFCache;
  // Stores the bottleneck of the residing task
  vector<vector<float> > BnkMat;
  ACT ACTFinal;
  void ComputeAvailableTreeFinal();
  void ComputeAvailableTreeFinalHelper(uint p, Tree<uint64_t>* t, uint d);
  uint64_t CalculateSpanFinal(int ResourceType);
  unsigned CalculateGroupSpanFinal(vector<int> & ResourcesVector);
  unsigned CalculateIssueSpanFinal(vector<int> & ResourcesVector);
  bool IsEmptyLevelFinal(unsigned ExecutionResource, uint64_t Level);
  unsigned GetGroupSpanFinal(vector<int> & ResourcesVector);
  unsigned GetLatencySpanFinal(unsigned i);
  unsigned CalculateLatencyOnlySpanFinal(unsigned i);
  unsigned GetGroupOverlapCyclesFinal(vector<int> & ResourcesVector);
unsigned GetOneToAllOverlapCyclesFinal(vector < int >&ResourcesVector);
unsigned GetOneToAllOverlapCyclesFinal(vector < int >&ResourcesVector, bool Issue);
  unsigned GetLatencyIssueOverlap(unsigned i);
  //---------------- CONTECH----------------------

  

  void checkUnique();
  //Statistics
  double AverageILP;
  

  //map <InstructionValueInstance, InstructionValueInstanceInfo> InstructionValueInstanceMap;
  map <InstructionValueInstance, uint64_t> InstructionValueInstanceMap;
  map <InstructionValue, int64_t> InstructionValueMap;
  map <uint64_t,PointerToMemoryInstance> AddressMap;

  vector <PointerToMemory> PointersToMemoryGlobalVector;
  vector <InstructionValueInstanceInfo> InstructionValueInstanceInfoGlobalVector;

/*
  map <PointerToMemory, vector< pair < InstructionValue, int64_t >>> PointerToMemoryInstructionsMap;
  map <PointerToMemory, uint64_t> PointerToMemoryAddressMap;

  map <Value*, vector< pair < Value*, uint64_t >>> PointerToMemoryInstructionsMap;
  map <Value*, uint64_t> PointerToMemoryAddressMap;
*/

  //map <Value*, int64_t> InstructionValueInstanceMap;
  map <Value*, Value*> InstructionValueInstructionNameMap;
  map <Value*, uint64_t> InstructionValueIssueCycleMap;
  map <uint64_t , CacheLineInfo> CacheLineIssueCycleMap;
  map <uint64_t , uint64_t> MemoryAddressIssueCycleMap;
  

#ifdef INTERMEDIATE_RESULTS_STACK
  deque<PointerToMemoryInstance> ReuseStack;
#else
  deque<uint64_t> ReuseStack;
#endif
  Tree<uint64_t> * ReuseTree;
  Tree<uint64_t> * PrefetchReuseTree;
  uint64_t PrefetchReuseTreeSize;
  double ErrorApproximationReuse;

  map <unsigned , pair<double,unsigned> > OverlapPercentagesMap;
  
  vector< vector<unsigned> > ParallelismDistribution;
  map<int,int> ReuseDistanceDistribution;
  map<int,map<uint64_t,uint> > ReuseDistanceDistributionExtended;
  
#ifdef SOURCE_CODE_ANALYSIS
  unordered_map<uint64_t,set<uint64_t> > SourceCodeLineOperations;
  unordered_map<uint64_t,set<uint64_t> > SourceCodeLineInfo;
  unordered_map<uint64_t,vector<uint64_t> > SourceCodeLineInfoBreakdown;
#endif
  
  //Constructor
  DynamicAnalysis(string TargetFunction,
                  string Microarchitecture,
                  unsigned MemoryWordSize,
                  unsigned CacheLineSize,
			   unsigned RegisterFileSize,
                  unsigned L1CacheSize,
                  unsigned L2CacheSize,
                  unsigned LLCCacheSize,
                  vector<float> ExecutionUnitsLatency,
                  vector<double> ExecutionUnitsThroughput,
                  vector<int> ExecutionUnitsParallelIssue,
                  vector<unsigned>  MemAccessGranularity,
                  int AddressGenerationUnits,
                  int InstructionFetchBandwidth,
                  int ReservationStationSize,
                  int ReorderBufferSize,
                  int LoadBufferSize,
                  int StoreBufferSize,
                  int LineFillBufferSize,
                  bool WarmCache,
                  bool x86MemoryModel,
                  bool SpatialPrefetcher,
                  bool ConstraintPorts,
			   bool ConstraintPortsx86,

                  bool ConstraintAGUs,
                  int rep,
                  bool InOrderExecution,
                  bool ReportOnlyPerformance,
                  unsigned PrefetchLevel,
                  unsigned PrefetchDispatch,
                  unsigned PrefetchTarget,
				  string OutputDir,
				  bool FloatPrecision);
  
  
  
  vector<Instruction*> instructionPool;
  
  
// ==================================================================================================
// 	FUNCTIONS for tracking pointers to memory - Implemented in ValuesAnalysis
//===================================================================================================

  void removePointerToMemoryInGlobalVector(int64_t PTMindex);
  int64_t insertPointerToMemoryInGlobalVector(PointerToMemory PTM);

  void printInstructionValueInstance(InstructionValueInstance IVI);
  void printPointerToMemoryInstance(PointerToMemoryInstance PTMI);
  void insertInstructionValueInstanceInfo(InstructionValueInstance IVI, InstructionValueInstanceInfo IVII);
  InstructionValueInstanceInfo getInstructionValueInstanceInfo(InstructionValueInstance IVI);
  InstructionValueInstanceInfo getInstructionValueInstanceInfo(int64_t Address);
  bool insertUsesOfPointerToMemory(Value * v, PointerToMemoryInstance PTMI, bool recursiveCall = false);
  PointerToMemoryInstance managePointerToMemory(Instruction & I, unsigned valueRep, int64_t valueInstance, uint64_t & addr, unsigned OpCode, bool WarmRun, bool forceAnalyze);
  unsigned adjustMemoryAddress(unsigned addr, unsigned addrFound, bool forceAnalyze);
  void resetInstructionValueMap();
  PointerToMemoryInstance getPointerToMemoryInstance(uint64_t address);
  int64_t getInstructionValueInstance(InstructionValue instValue);
  int64_t insertInstructionValueInstanceInfoGlobalVector(InstructionValueInstanceInfo IVII);
  uint64_t getInstructionValueInstanceInfoGlobalVectorIndex(PointerToMemoryInstance PTMI);
  int64_t getInstructionValueInstanceInfoGlobalVectorIndex(InstructionValueInstance IVI);

  InstructionValueInstance getInstructionValueInstance(PointerToMemoryInstance PTMI);
  InstructionValueInstance getInstructionValueInstance(uint64_t MemoryAddress);
  void increaseInstructionValueInstance(InstructionValue v);
  void printInstructionValue(InstructionValue IV);
  void printPointerToMemory (PointerToMemory ptrmem);
  void printPointerToMemoryGlobalVector();
  void printInstructionValueInstanceInfo(InstructionValueInstanceInfo IVII);
  void printInstructionValueInstanceMap();
  void printAddressMap();
  void printInstructionValueInstanceInfoGlobalVector();
  void insertAddressMap(uint64_t Address, PointerToMemoryInstance PTMI);
  void setAddress(uint64_t InstructioValueInfoIndex, uint64_t Address);
  void mergeAddressMap();

  PointerToMemoryInstance getPointerToMemoryToInsertInRegisterStack(Value * I, int64_t valueInstance, bool intermediateResult);
  void insertIntermediateResultInRegisterStack(PointerToMemoryInstance PTMI, Instruction & currentInstruction, bool WarmRun, bool isSpill = false);
  int RegisterStackReuseDistance(PointerToMemoryInstance address, Instruction & CurrentInst, bool WarmRun,  bool isSpill);
  void insertRegisterStack(PointerToMemoryInstance address, Instruction & CurrentInst, bool warmRun);
  void removeRegisterStack(PointerToMemoryInstance address);
  void printRegisterStack();

  void increaseNUses(PointerToMemoryInstance PTMI);
  void decreaseNUses(PointerToMemoryInstance PTMI);

  // ==================================================================================================
  // 	OTHER FUNCTIONS
  //===================================================================================================


  void analyze();
#ifdef INTERPRETER
  void analyzeInstruction(Instruction &I, ExecutionContext &SF,  GenericValue * visitResult, unsigned valueRep = 0);
#else
  void analyzeInstruction (Instruction &I, unsigned OpCode, uint64_t addr, unsigned SourceCodeLine, bool forceAnalyze = false, unsigned VectorWidth = 1, unsigned valueRep = 0, bool lastValue= true, bool isSpill = false);
#endif
  
  void insertInstructionValueIssueCycle(Value* v,uint64_t InstructionIssueCycle, bool isPHINode = 0 );
  void insertCacheLineLastAccess(uint64_t v,uint64_t LastAccess );
  void insertCacheLineInfo(uint64_t v,CacheLineInfo Info );
  void insertMemoryAddressIssueCycle(uint64_t v,uint64_t Cycle );
  
  
  unsigned getLastRepetitionIntrinsic(string functionName);
  void getOperandsPositionsIntrinsic(string functionName, vector<unsigned> & positions);
  unsigned getStoreOperandPositionIntrinsic(string functionName);

  uint64_t getInstructionValueIssueCycle(Value* v);
  void insertInstructionValueName (Value * v);
  void insertUseInPointerToMemoryInstructions(PointerToMemory ptrmem, InstructionValue v, int64_t valueInstance);
  void printInstructionValueNames();
  void printInstructionValueInstances();
  uint64_t getCacheLineLastAccess(uint64_t v);
  CacheLineInfo getCacheLineInfo(uint64_t v);
  uint64_t getMemoryAddressIssueCycle(uint64_t v);

  void printPointerToMemoryAddressMap();
  void printPointerToMemoryInstructionsMap();





  unsigned adjustMemoryAddress(PointerToMemory v, unsigned addr, bool forceAnalyze);
  Value * getLastUsePhiNode(Value * phiNodeValue);
  void getOriginalIncomingEdgesPhiNode(PHINode * phiNode, vector<Value *>&  originalIncomingEdges);
///  vector<PointerToMemory> getPointerToMemoryToInsertInRegisterStack(Value * I, int64_t valueInstance, bool intermediateResults);
  PointerToMemory getPointerToMemoryAddress(uint64_t Address);
  void insertPointerToMemoryAddress(PointerToMemory ptrmem, uint64_t Address);
  void insertConditionalPointerToMemoryAddress(PointerToMemory ptrmem, uint64_t Address);
  void insertPointerToMemoryInstructions(Value * v,unsigned valueRep,   PointerToMemory ptrmemOperand = {NULL, NULL, NULL}, bool isGetElementPtr = false);
  void insertPointerToMemoryInstructionsBitCast(Instruction &I, unsigned valueRep);
  PointerToMemory  getPointerToMemoryInstructions(Value * v, unsigned valueRep, int64_t valueInstance);
/*
  void insertPointerToMemoryAddress(Value * v, uint64_t Address);
  void insertConditionalPointerToMemoryAddress(Value * v, uint64_t Address);
  void insertPointerToMemoryInstructions(Value * v, Value * vOperand = NULL, bool isGetElementPtr = false);
  void insertPointerToMemoryInstructionsBitCast(Instruction &I);
  Value *  getPointerToMemoryInstructions(Value * v);
*/
  unsigned  GetInstructionTypeFromPrefetchType(unsigned PrefetchType);
  
  
  uint64_t GetLastIssueCycle(unsigned ExecutionResource, bool WithPrefetch = false);
#ifdef EFF_TBV
  void GetTreeChunk (uint64_t i, unsigned int ExecutionResource);
#else
  uint64_t GetTreeChunk(uint64_t i);
#endif
  
  
  float GetEffectiveThroughput(unsigned ExecutionResource, unsigned AccessWidth, unsigned NElementsVector);
  unsigned GetIssueCycleGranularity(unsigned ExecutionResource, unsigned AccessWidth, unsigned NElementsVector);
  unsigned GetNodeWidthOccupancy(unsigned ExecutionResource, unsigned AccessWidth, unsigned NElementsVector);
  bool GetLevelFull(unsigned ExecutionResource, unsigned AccessWidth, unsigned NElementsVector, unsigned NodeIssueOccupancy, unsigned NodeWidthOccupancy, bool potentiallyFull = false);
  bool GetEnoughWidthOccupancy(unsigned ExecutionResource, unsigned NodeWidth, unsigned AccessWidth, unsigned NElementsVector);

  vector<unsigned> IssuePorts;

  //Returns the DAG level occupancy after the insertion
  unsigned FindNextAvailableIssueCycle(unsigned OriginalCycle, unsigned ExecutionResource, uint8_t NElementsVector = 1, bool TargetLevel = true);
  unsigned FindNextAvailableIssueCyclePortAndThroughtput(unsigned InstructionIssueCycle, unsigned ExtendedInstructionType, unsigned NElementsVector=1);
  
  bool ThereIsAvailableBandwidth(unsigned NextAvailableCycle, unsigned ExecutionResource, unsigned NElementsVector, bool& FoundInFullOccupancyCyclesTree, bool TargetLevel);
  
  uint64_t FindNextAvailableIssueCycleUntilNotInFullOrEnoughBandwidth(unsigned NextCycle, unsigned ExecutionResource , bool& FoundInFullOccupancyCyclesTree, bool& EnoughBandwidth);
  
  bool InsertNextAvailableIssueCycle(uint64_t NextAvailableCycle, unsigned ExecutionResource, unsigned NElementsVector = 1, int IssuePort = -1, bool isPrefetch=0);
  
  void IncreaseInstructionFetchCycle(bool EmptyBuffers = false);
  
  unsigned CalculateIssueCycleGranularity(unsigned ExecutionResource, unsigned NElementsVector=1);
  
  uint64_t CalculateSpan(int ResourceType);
  //unsigned CalculateGroupSpan(int NResources, ...);
  unsigned CalculateGroupSpan(vector<int> & ResourcesVector, bool WithPrefetch = false, bool ForceUnitLatency = false);

  
  unsigned CalculateIssueSpan(vector<int> & ResourcesVector);
  
  
  unsigned CalculateGroupSpanUnitLatency(vector<int> & ResourcesVector, bool ForceUnitLatency = false);
  
  unsigned CalculateResourceStallSpan(int resource, int stall);
  void CalculateResourceStallOverlapCycles(Tree<uint64_t> * n, int resource, uint64_t & OverlapCycles);
  
  
  void CollectSourceCodeLineStatistics(uint64_t ResourceType, uint64_t Cycle,  uint64_t MaxLatencyLevel, uint64_t SpanIncrease, bool IsInFullOccupancyCyclesTree, bool IsInAvailableCyclesTree);
  
  bool IsEmptyLevel(unsigned ExecutionResource, uint64_t Level, bool& IsInAvailableCyclesTree,
                    bool& IsInFullOccupancyCyclesTree , bool WithPrefetch = false);
  
  
  uint64_t FindNextNonEmptyLevel(unsigned ExecutionResource, uint64_t Level);
  bool isStallCycle(int ResourceType, uint64_t Level);
  
  
  unsigned GetMemoryInstructionType(int ReuseDistance, uint64_t MemoryAddress,bool isLoad=true);
  unsigned GetExtendedInstructionType(Instruction &I, int OpCode, int ReuseDistance=0, int RegisterStackReuseDistance = -1);
  unsigned GetPositionSourceCodeLineInfoVector(uint64_t Resource);
  
  uint64_t GetMinIssueCycleReservationStation();
  uint64_t GetMinCompletionCycleLoadBuffer();
  uint64_t GetMinCompletionCycleLoadBufferTree();
  
  uint64_t GetMinCompletionCycleStoreBuffer();
  uint64_t GetMinCompletionCycleLineFillBuffer();
  
  void RemoveFromReservationStation(uint64_t Cycle);
  void RemoveFromReorderBuffer(uint64_t Cycle);
  void RemoveFromLoadBuffer(uint64_t Cycle);
  void RemoveFromLoadBufferTree(uint64_t Cycle);
  
  void RemoveFromStoreBuffer(uint64_t Cycle);
  void RemoveFromLineFillBuffer(uint64_t Cycle);
  
  void RemoveFromDispatchToLoadBufferQueue(uint64_t Cycle);
  void RemoveFromDispatchToLoadBufferQueueTree(uint64_t Cycle);
  void RemoveFromDispatchToStoreBufferQueue(uint64_t Cycle);
  void RemoveFromDispatchToLineFillBufferQueue(uint64_t Cycle);
  
  ComplexTree<uint64_t> * RemoveFromDispatchAndInsertIntoLoad(uint64_t i, ComplexTree<uint64_t> * t);
  void inOrder(uint64_t i, ComplexTree<uint64_t> * n);
  
  void DispatchToLoadBuffer(uint64_t Cycle);
  void DispatchToLoadBufferTree(uint64_t Cycle);
  
  void DispatchToStoreBuffer(uint64_t Cycle);
  void DispatchToLineFillBuffer(uint64_t Cycle);
  
  uint64_t FindIssueCycleWhenLoadBufferIsFull();
  uint64_t FindIssueCycleWhenLoadBufferTreeIsFull();
   uint64_t FindIssueCycleWhenLoadBufferTreeIsFullOld();
  
  uint64_t FindIssueCycleWhenStoreBufferIsFull();
  uint64_t FindIssueCycleWhenLineFillBufferIsFull();
  
  string GetResourceName(unsigned Resource);
  string GetNodeName(unsigned Node);
  
  void PrintReorderBuffer();
  void PrintReservationStation();
  void PrintLoadBuffer();
  void PrintLoadBufferTreeRecursive(SimpleTree<uint64_t> * p);
  void PrintDispatchToLoadBufferTreeRecursive(ComplexTree<uint64_t> * p, bool key);
  void PrintDispatchToLoadBufferTree();
  
  void PrintLoadBufferTree();
  void PrintStoreBuffer();
  void PrintLineFillBuffer();
  void PrintDispatchToStoreBuffer();
  void PrintDispatchToLoadBuffer();
  void PrintDispatchToLineFillBuffer();
  
#ifdef INTERMEDIATE_RESULTS_STACK


//  int RegisterStackReuseDistance(PointerToMemory address, Instruction & CurrentInst);
//  void insertRegisterStack(PointerToMemory address, Instruction & CurrentInst, bool warmRun);

  //void insertIntermediateResultInRegisterStack(Value * I,  int64_t valueInstance, Instruction & currentInstruction, bool WarmWun);
  int64_t insertOperandsInRegisterStack(unsigned operandPosition, Instruction & I, bool WarmCache, unsigned Line);
  void removeOperandsInRegisterStack(unsigned i, Instruction & I, bool WarmRun, unsigned Line);
#else
  int RegisterStackReuseDistance(uint64_t address, bool WarmCache, bool insert = true);
  void insertRegisterStack(uint64_t address);
#endif
  int ReuseDistance(uint64_t Last, uint64_t Current, uint64_t address, bool FromPrefetchReuseTree = false);
  int ReuseTreeSearchDelete(uint64_t Current, uint64_t address,  bool FromPrefetchReuseTree = false);
  //int ReuseTreeSearchDelete(uint64_t Last, bool FromPrefetchReuseTree = false);
  void updateReuseDistanceDistribution(int Distance, uint64_t InstructionIssueCycle);
  unsigned int roundNextPowerOfTwo(unsigned int v);
  unsigned int roundNextMultiple(uint64_t num, int multiple);
  unsigned int roundNextMultipleOf2(uint64_t num);
  unsigned int DivisionRoundUp(float a, float b);
  void finishAnalysis();
  void finishAnalysisContechSimplified();
  void finishAnalysisContech(bool isBnkReqd);
  
  void printHeaderStat(string Header);
  
  int getInstructionType(Instruction &I);
  int getInstructionComputationDAGNode(Instruction &I);
  
  void dumpList(std::list< double > const & l, string const & filename);


};
#endif
