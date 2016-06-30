//=-------------------- llvm/Support/DynamicAnalysis.h ------======= -*- C++ -*//
//
//                     The LLVM Compiler Infrastructure
//
//  Victoria Caparros Cabezas <caparrov@inf.ethz.ch>
//===----------------------------------------------------------------------===//

#define INTERPRETER

#ifdef INTERPRETER
#include "llvm/Support/DynamicAnalysis.h"
#else
#include "DynamicAnalysis.h"
#endif

//===----------------------------------------------------------------------===//
//                        Constructor of the analyzer
//===----------------------------------------------------------------------===//

DynamicAnalysis::DynamicAnalysis (string TargetFunction,
                                  string Microarchitecture,
                                  unsigned MemoryWordSize,
                                  unsigned CacheLineSize,
                                  unsigned RegisterFileSize,
                                  unsigned L1CacheSize,
                                  unsigned L2CacheSize,
                                  unsigned LLCCacheSize,
                                  vector < float >ExecutionUnitsLatency,
                                  vector < double >ExecutionUnitsThroughput,
                                  vector < int >ExecutionUnitsParallelIssue,
                                  vector < unsigned >MemAccessGranularity,
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
						    unsigned PrefetchTarget)
{
  
  
  vector < unsigned >emptyVector;
  
  // =================== Sandy Bridge config ================================//
  
  if (Microarchitecture.compare ("SB") == 0) {
    uarch = SANDY_BRIDGE;
    nExecutionUnits = SANDY_BRIDGE_EXECUTION_UNITS;
    nArithmeticExecutionUnits = SANDY_BRIDGE_ARITHMETIC_EXECUTION_UNITS;
    nMovExecutionUnits = SANDY_BRIDGE_MOV_EXECUTION_UNITS;
    nMemExecutionUnits = SANDY_BRIDGE_MEM_EXECUTION_UNITS;
    
    nArithmeticNodes = SANDY_BRIDGE_ARITHMETIC_NODES;
    nMovNodes = SANDY_BRIDGE_MOV_NODES;
    nMemNodes = SANDY_BRIDGE_MEM_NODES;
    nPorts = SANDY_BRIDGE_DISPATCH_PORTS;
    nBuffers = SANDY_BRIDGE_BUFFERS;
    nAGUs = SANDY_BRIDGE_AGUS;
    nLoadAGUs = SANDY_BRIDGE_LOAD_AGUS;
    nStoreAGUs = SANDY_BRIDGE_STORE_AGUS;
    nPrefetchNodes = SANDY_BRIDGE_PREFETCH_NODES;
    nNodes = SANDY_BRIDGE_NODES;
    
    // Mapping between nodes and execution units. ExecutionUnit[] vector contains
    // one entry for every type of node, and the associated execution unit.
    
    for (unsigned i = 0; i < nNodes; i++)
      ExecutionUnit.push_back (0);
    
    
    ExecutionUnit[FP_ADD_NODE] = FP_ADDER;
    ExecutionUnit[FP_MUL_NODE] = FP_MULTIPLIER;
    ExecutionUnit[FP_DIV_NODE] = FP_DIVIDER;
    
    ExecutionUnit[FP_SHUFFLE_NODE] = FP_SHUFFLE_UNIT;
    ExecutionUnit[FP_BLEND_NODE] = FP_BLEND_UNIT;
    ExecutionUnit[FP_MOV_NODE] = FP_MOV_UNIT;
    
    
    
    ExecutionUnit[REGISTER_STORE_NODE] = REGISTER_STORE_CHANNEL;
    ExecutionUnit[REGISTER_LOAD_NODE] =REGISTER_LOAD_CHANNEL;
    ExecutionUnit[L1_STORE_NODE] = L1_STORE_CHANNEL;
    ExecutionUnit[L1_LOAD_NODE] = L1_LOAD_CHANNEL;
    ExecutionUnit[L2_STORE_NODE] = L2_STORE_CHANNEL;
    ExecutionUnit[L2_LOAD_NODE] = L2_LOAD_CHANNEL;
    ExecutionUnit[L3_STORE_NODE] = L3_STORE_CHANNEL;
    ExecutionUnit[L3_LOAD_NODE] = L3_LOAD_CHANNEL;
    ExecutionUnit[MEM_STORE_NODE] = MEM_STORE_CHANNEL;
    ExecutionUnit[MEM_LOAD_NODE] = MEM_LOAD_CHANNEL;
    
    ExecutionUnit[AGU_NODE] = ADDRESS_GENERATION_UNIT;
    
    ExecutionUnit[PORT_0_NODE] = PORT_0;
    ExecutionUnit[PORT_1_NODE] = PORT_1;
    ExecutionUnit[PORT_2_NODE] = PORT_2;
    ExecutionUnit[PORT_3_NODE] = PORT_3;
    ExecutionUnit[PORT_4_NODE] = PORT_4;
    ExecutionUnit[PORT_5_NODE] = PORT_5;
    
  }
  
  // =================== Atom config ================================//
  
  if (Microarchitecture.compare ("ATOM") == 0) {
    uarch = ATOM;
    nExecutionUnits = ATOM_EXECUTION_UNITS;
    nPorts = ATOM_DISPATCH_PORTS;
    nBuffers = ATOM_BUFFERS;
    nAGUs = ATOM_AGUS;
    nLoadAGUs = ATOM_LOAD_AGUS;
    nStoreAGUs = ATOM_STORE_AGUS;
  }
  
  
  // =================== Some general checkings ================================//
  
  if (!ExecutionUnitsLatency.empty () && ExecutionUnitsLatency.size () != nExecutionUnits) {
    dbgs() << "ExecutionUnitsLatency.size () " << ExecutionUnitsLatency.size () << "\n";
    dbgs() << "nExecutionUnits " << nExecutionUnits<< "\n";
    report_fatal_error ("The number of latencies does not match the number of execution units");
    
  }
  if (!ExecutionUnitsThroughput.empty ()
      && ExecutionUnitsThroughput.size () != nExecutionUnits)
    report_fatal_error ("The number of throughputs does not match the number of execution units");
  
  if (!ExecutionUnitsParallelIssue.empty ()
      && ExecutionUnitsParallelIssue.size () != nExecutionUnits)
    report_fatal_error ("The number of execution units parallel issue does not match the number of execution units");
  if (L1CacheSize != 0 && L1CacheSize < CacheLineSize)
    report_fatal_error ("L1 cache size < cache line size");
  if (L2CacheSize != 0 && L2CacheSize < CacheLineSize)
    report_fatal_error ("L2 cache size < cache line size");
  if (LLCCacheSize != 0 && LLCCacheSize < CacheLineSize)
    report_fatal_error ("LLC cache size < cache line size");
  
  if (CacheLineSize % MemoryWordSize != 0)
    report_fatal_error ("Cache line size is not a multiple of memory word size");
  

  // Initialize local variables with command-line arguemtns
  this->TargetFunction = TargetFunction;
  this->MemoryWordSize = MemoryWordSize;
  this->CacheLineSize = CacheLineSize / (this->MemoryWordSize);
  
  
  this->RegisterFileSize = RegisterFileSize;
  // If caches sizes are not multiple of a power of 2, force it.
  //this->L1CacheSize = roundNextPowerOfTwo(L1CacheSize);
  //this->L2CacheSize = roundNextPowerOfTwo(L2CacheSize);
  //this->LLCCacheSize = roundNextPowerOfTwo(LLCCacheSize);
  this->L1CacheSize = L1CacheSize / CacheLineSize;
  this->L2CacheSize = L2CacheSize / CacheLineSize;
  this->LLCCacheSize = LLCCacheSize / CacheLineSize;
  
  
  this->AddressGenerationUnits = AddressGenerationUnits;
  this->ReservationStationSize = ReservationStationSize;
  this->InstructionFetchBandwidth = InstructionFetchBandwidth;
  this->ReorderBufferSize = ReorderBufferSize;
  this->LoadBufferSize = LoadBufferSize;
  this->StoreBufferSize = StoreBufferSize;
  this->LineFillBufferSize = LineFillBufferSize;
  this->WarmCache = WarmCache;
  this->x86MemoryModel = x86MemoryModel;
  this->SpatialPrefetcher = SpatialPrefetcher;
  this->ConstraintPorts = ConstraintPorts;
  this->ConstraintPortsx86 = ConstraintPortsx86;
  // If ConstraintPortsx86 is true, ConstraintPorts must be also true.
  if(ConstraintPortsx86)
    this->ConstraintPorts = true;
  this->ConstraintAGUs = ConstraintAGUs;
  this->InOrderExecution = InOrderExecution;
  this->ReportOnlyPerformance = ReportOnlyPerformance;
  this->rep = rep;
  
  SmallBuffers = false;
  /*
   if(LoadBufferSize < 100)
   SmallBuffers = true;
   else{
   SmallBuffers = false;
   LoadBufferCompletionCyclesTree = NULL;
   DispatchToLoadBufferQueueTree = NULL;
   }
   */
  
  LoadBufferCompletionCyclesTree = NULL;
  DispatchToLoadBufferQueueTree = NULL;
  
  MinLoadBuffer = 0;
  MaxDispatchToLoadBufferQueueTree = 0;
  
  BitsPerCacheLine = log2 (this->CacheLineSize * (this->MemoryWordSize));
  
  NRegisterSpills = 0;
  
  // In reality is if L2, but need to specify the size for the reuse disrance
  switch (PrefetchLevel) {
    case 1:
      this->PrefetchLevel = 0;
      PrefetchDestination = L1_LOAD_CHANNEL;
      break;
    case 2:
      this->PrefetchLevel = this->L1CacheSize;
      PrefetchDestination = L2_LOAD_CHANNEL;
      break;
    case 3:
      this->PrefetchLevel = this->L2CacheSize;
      PrefetchDestination = L3_LOAD_CHANNEL;
      break;
    default:
      report_fatal_error ("Prefetch level not recognized");
      break;
  }
  
  
  switch (PrefetchDispatch) {
    case 0:
      this->PrefetchDispatch = 0;
      break;
    case 1:
      this->PrefetchDispatch = L1_STORE_NODE;
      break;
    case 2:
      this->PrefetchDispatch = L2_STORE_NODE;
      break;
    case 3:
      this->PrefetchDispatch = L3_STORE_NODE;
      break;
    default:
      report_fatal_error ("Prefetch dispatch not recognized");
      break;
  }
  
  switch (PrefetchTarget) {
    case 2:
      this->PrefetchTarget = L1_STORE_CHANNEL;
      break;
    case 3:
      this->PrefetchTarget = L2_STORE_CHANNEL;
      break;
    case 4:
      this->PrefetchTarget = L3_STORE_CHANNEL;
      break;
    default:
      report_fatal_error ("Prefetch target not recognized");
      break;
  }
  
  VectorWidth = 4;
  SourceCodeLine = 0;
  
  
  
  if (!MemAccessGranularity.empty ()
      && MemAccessGranularity.size () != nMemExecutionUnits)
    report_fatal_error ("Mem access granularities do not match the number of memory execution units");
  
  // We have latency and throughput for every resource in which we schedule nodes.
  // Latency and throughput of execution resources can be specified via command line.
  for (unsigned i = 0; i < nExecutionUnits; i++) {
    this->ExecutionUnitsLatency.push_back (1);	//Default value for latency
    this->ExecutionUnitsThroughput.push_back (-1);	// Infinite throughput
    this->ExecutionUnitsParallelIssue.push_back (-1);	// Default value of parallel issue is 1.
    
    if (i < nArithmeticExecutionUnits + nMovExecutionUnits) {
      AccessGranularities.push_back (1);
    }
    else {
      AccessGranularities.push_back (this->MemoryWordSize);
    }
    
    vector < float >BnkVec;
    for (unsigned j = 0; j < nBuffers + 2; ++j) {
      BnkVec.push_back (INF);
    }
    this->BnkMat.push_back (BnkVec);
    
  }
  
  
  MaxLatencyResources = 0;
  if (!ExecutionUnitsLatency.empty ()) {
    for (unsigned i = 0; i < nExecutionUnits; i++){
      this->ExecutionUnitsLatency[i] = ceil (ExecutionUnitsLatency[i]);
      MaxLatencyResources = max(MaxLatencyResources,this->ExecutionUnitsLatency[i] );
    }
    RARDependences = true;
  }else{
    RARDependences = false;
    for (unsigned i = 0; i < nExecutionUnits; i++){
      MaxLatencyResources = max(MaxLatencyResources,this->ExecutionUnitsLatency[i] );
    }
    
  }
  
  
  if (!ExecutionUnitsThroughput.empty ())
    for (unsigned i = 0; i < nExecutionUnits; i++)
      
      this->ExecutionUnitsThroughput[i] = ExecutionUnitsThroughput[i];
  
  if (!ExecutionUnitsParallelIssue.empty ()){
    
    for (unsigned i = 0; i < nExecutionUnits; i++)
      this->ExecutionUnitsParallelIssue[i] = ExecutionUnitsParallelIssue[i];
  }
  
  if (!MemAccessGranularity.empty ())
    for (unsigned i = 0; i < nMemExecutionUnits; i++)
      AccessGranularities[i + nArithmeticExecutionUnits + nMovExecutionUnits] = MemAccessGranularity[i];
  

  // Mapping between nodes and dispatch ports
  // IMPORTANT: Associate Dispatch ports to nodes instead of execution resources. Because
  // otherwise there is a problem when different nodes that share execution unit
  // but no dispatch ports
  
  for (unsigned i = 0; i < nArithmeticNodes + nMovNodes + nMemNodes; i++)
    DispatchPort.push_back (emptyVector);
  
  
  
  /*
   Port mapping in Sandy Bridge
   Port 0 -> FP_MUL, FP_DIV, FP_BLEND, FP_MOV
   Port 1 -> FP_ADD
   Port 2 -> LOAD (L1, L2, L3 and MEM)
   Port 3 -> LOAD (L1, L2, L3 and MEM)
   Port 4 -> STORE_CHANNEL (L1, L2, L3 and MEM)
   Port 5 -> FP_SHUFFLE, FP_BLEND, FP_MOV
   
   */
  
  if(ConstraintPortsx86){
    
    emptyVector.push_back (PORT_1);
    DispatchPort[FP_ADD_NODE] = emptyVector;
    
    emptyVector.clear ();
    emptyVector.push_back (PORT_0);
    DispatchPort[FP_MUL_NODE] = emptyVector;
    
    emptyVector.clear ();
    emptyVector.push_back (PORT_0);
    DispatchPort[FP_DIV_NODE] = emptyVector;
    
    emptyVector.clear ();
    emptyVector.push_back (PORT_5);
    DispatchPort[FP_SHUFFLE_NODE] = emptyVector;
    
    
    emptyVector.clear ();
    emptyVector.push_back (PORT_0);
    emptyVector.push_back (PORT_5);
    DispatchPort[FP_MOV_NODE] = emptyVector;
    
    emptyVector.clear ();
    emptyVector.push_back (PORT_0);
    emptyVector.push_back (PORT_5);
    DispatchPort[FP_BLEND_NODE] = emptyVector;
    
    
    // Registers don't have any associated dispatch port
    emptyVector.clear ();
    DispatchPort[REGISTER_LOAD_NODE] = emptyVector;
    DispatchPort[REGISTER_STORE_NODE] = emptyVector;
    
    emptyVector.clear ();
    emptyVector.push_back (PORT_4);
    DispatchPort[L1_STORE_NODE] = emptyVector;
    DispatchPort[L2_STORE_NODE] = emptyVector;
    DispatchPort[L3_STORE_NODE] = emptyVector;
    DispatchPort[MEM_STORE_NODE] = emptyVector;
    
    emptyVector.clear ();
    emptyVector.push_back (PORT_2);
    emptyVector.push_back (PORT_3);
    DispatchPort[L1_LOAD_NODE] = emptyVector;
    DispatchPort[L2_LOAD_NODE] = emptyVector;
    DispatchPort[L3_LOAD_NODE] = emptyVector;
    DispatchPort[MEM_LOAD_NODE] = emptyVector;
    
    
    // ConstraintPortsx86 forces some conditions like divisions and multiplications are
    // issued in the same port. If parallel issue of a given node is larger than the
    // default ports in x86, then we add ports to the corresponding pool of ports in
    // DispathPort.
    
    for (unsigned i = 0; i < nArithmeticNodes + nMovNodes; i++){
      
      if (this->ExecutionUnitsParallelIssue[ExecutionUnit[i]] != INF){
        if((unsigned)this->ExecutionUnitsParallelIssue[ExecutionUnit[i]] > DispatchPort[i].size()){
          unsigned initialPortsSize = DispatchPort[i].size();
          for (unsigned j = initialPortsSize; j < (unsigned)this->ExecutionUnitsParallelIssue[ExecutionUnit[i]]; j++){
            nPorts++;
            DispatchPort[i].push_back(PORT_0+nPorts-1);
          }
        }
      }else{
        
        report_fatal_error ("ExecutionUnitsParallelIssue cannot be INF if ConstraintPortsx86");
      }
    }
    
    // Memory nodes - All load memory nodes (L1, ... ,mem) and all stores memory nodes
    // should have the same number of ports (except register!, hence starting the for with +2)
    for (unsigned i = nArithmeticNodes + nMovNodes+2; i < nArithmeticNodes + nMovNodes + nMemNodes; i++){
      if (this->ExecutionUnitsParallelIssue[ExecutionUnit[i]] != INF){
        if((unsigned)this->ExecutionUnitsParallelIssue[ExecutionUnit[i]] > DispatchPort[i].size()){
          unsigned initialPortsSize = DispatchPort[i].size();
          for (unsigned j = initialPortsSize; j < (unsigned)this->ExecutionUnitsParallelIssue[ExecutionUnit[i]]; j++){
            
            nPorts++;
            if (i == L1_LOAD_NODE || i == L2_LOAD_NODE || i == L3_LOAD_NODE || i == MEM_LOAD_NODE ){
              
              // The extra port is associated to all load nodes, because all loads should be issued
              // by the same port. Parallel issue determines how many can be done i parallel.
              //DispatchPort[i].push_back(PORT_0+nPorts-1);
              DispatchPort[L1_LOAD_NODE].push_back(PORT_0+nPorts-1);
              DispatchPort[L2_LOAD_NODE].push_back(PORT_0+nPorts-1);
              DispatchPort[L3_LOAD_NODE].push_back(PORT_0+nPorts-1);
              DispatchPort[MEM_LOAD_NODE].push_back(PORT_0+nPorts-1);
              
            }else{
              if (i == L1_STORE_NODE || i == L2_STORE_NODE || i == L3_STORE_NODE || i == MEM_STORE_NODE ){
                
                DispatchPort[L1_STORE_NODE].push_back(PORT_0+nPorts-1);
                DispatchPort[L2_STORE_NODE].push_back(PORT_0+nPorts-1);
                DispatchPort[L3_STORE_NODE].push_back(PORT_0+nPorts-1);
                DispatchPort[MEM_STORE_NODE].push_back(PORT_0+nPorts-1);
                
              }else{
                report_fatal_error ("Memory node not recognized\n");
              }
            }
          }
        }
        
      }else{
        report_fatal_error ("ExecutionUnitsParallelIssue cannot be INF if ConstraintPortsx86");
      }
    }
  }else{
    // If constraint ports but no x86 ports
    if (ConstraintPorts){
      
      // Initial number of ports
      for (unsigned i = 0; i <  nArithmeticNodes + nMovNodes + nMemNodes; i++) {
        emptyVector.clear ();
        if(this->ExecutionUnitsParallelIssue[ExecutionUnit[i]] != INF){
          for (unsigned j = 0; j< (unsigned)this->ExecutionUnitsParallelIssue[ExecutionUnit[i]]; j++){
            emptyVector.push_back (PORT_0+nPorts);
            nPorts++;
          }
          
        }
        DispatchPort[i] = emptyVector;
        
      }

    }else{
      // If no constraint ports
      // vector < int >emptyVectorInt;
      //emptyVector.push_back (0);
      
      emptyVector.clear();
      for (unsigned i = 0; i < nArithmeticNodes + nMovNodes + nMemNodes; i++){
        DispatchPort[i] = emptyVector;
      }
      
    }
    
  }
  
  
  for (unsigned i = 0; i < nArithmeticNodes + nMovNodes + nMemNodes; i++) {	// Dispatch ports are associated to nodes
    if (ConstraintPorts && this->ExecutionUnitsParallelIssue[ExecutionUnit[i]] > 0
        && DispatchPort[i].size () < (unsigned) this->ExecutionUnitsParallelIssue[ExecutionUnit[i]]) {
      if (i != REGISTER_LOAD_NODE && i != REGISTER_STORE_NODE){
        DEBUG (dbgs () << "ExecutionUnit " << i << "\n");
        DEBUG (dbgs () << "DispatchPort[i].size() " << DispatchPort[i].size () << "\n");
        DEBUG (dbgs () << "ExecutionUnitsParallelIssue[i] " << ExecutionUnitsParallelIssue[i] << "\n");
        report_fatal_error ("There are more execution units than ports that can dispatch them\n");
      }
    }
    
    if ((this->ExecutionUnitsParallelIssue[ExecutionUnit[i]]==INF && ConstraintPortsx86) && DispatchPort[i].size() != 0)
      report_fatal_error ("Parallel Issue cannot infinite while constraining ports\n");
  }
  
  // +2 because REGISTER_CHANNEL (load and store) does not require AGUs but are memory nodes
  for (unsigned i = nArithmeticNodes + nMovNodes+2; i <= nArithmeticNodes + nMovNodes + nMemNodes; i++){
    if(ConstraintAGUs && (this->ExecutionUnitsParallelIssue[ExecutionUnit[i]] == INF || this->ExecutionUnitsThroughput[ExecutionUnit[i]] == INF)){
      dbgs () << "ExecutionUnit " <<  GetResourceName(ExecutionUnit[i]) << "\n";
      dbgs () << "ExecutionUnitsThroughput " << this->ExecutionUnitsThroughput[ExecutionUnit[i]] << "\n";
      dbgs () << "ExecutionUnitsParallelIssue " << this->ExecutionUnitsParallelIssue[ExecutionUnit[i]] << "\n";
      dbgs() << "WARNING: Memory throughput (throughput or parallel issue) should be INF in constraintAGUs\n";
    }
  }
  
  
  
  
  
  
  // Latency and throughput of buffers. Although it has no effect, these
  // values are used
  for (unsigned i = 0; i < nBuffers; i++) {
    this->ExecutionUnitsLatency.push_back (1);	//Default value for latency
    this->ExecutionUnitsThroughput.push_back (1);	// Infinite throughput
    this->ExecutionUnitsParallelIssue.push_back (1);
    AccessGranularities.push_back (1);
  }
  
  
  // Does not matter whether we constraint AGUs or not, ExecutionUnitsLatency,
  // ExecutionUnitsParallelIssue and  ExecutionUnitsThroughput have an entry
  // for them. If constraint is false, then thorughput is set to infinity.
  /*
   if (ConstraintAGUs== false){
   nAGUs = 0;
   nLoadAGUs = 0;
   nStoreAGUs = 0;
   }
   */
  
  // Latency and throughput of AGUs
  if (nAGUs > 0) {
    this->ExecutionUnitsLatency.push_back (1);
    if (ConstraintAGUs) {
      this->ExecutionUnitsThroughput.push_back (1);
      this->ExecutionUnitsParallelIssue.push_back (SANDY_BRIDGE_AGUS);
    }
    else {
      this->ExecutionUnitsThroughput.push_back (-1);
      this->ExecutionUnitsParallelIssue.push_back (-1);
    }
    AccessGranularities.push_back (1);
  }
  
  if (nLoadAGUs > 0) {
    this->ExecutionUnitsLatency.push_back (1);
    this->ExecutionUnitsThroughput.push_back (1);
    this->ExecutionUnitsParallelIssue.push_back (nLoadAGUs);
    AccessGranularities.push_back (1);
  }
  
  if (nStoreAGUs > 0) {
    this->ExecutionUnitsLatency.push_back (1);
    this->ExecutionUnitsThroughput.push_back (1);
    this->ExecutionUnitsParallelIssue.push_back (nStoreAGUs);
    AccessGranularities.push_back (1);
  }
  
  // Latency and throughput of ports
  for (unsigned i = 0; i < nPorts; i++) {
    this->ExecutionUnitsLatency.push_back (1);	//Default value for latency
    if (ConstraintPorts) {
      this->ExecutionUnitsThroughput.push_back (1);
      this->ExecutionUnitsParallelIssue.push_back (1);
    }else {
      this->ExecutionUnitsThroughput.push_back (-1);	// Infinite throughput
      this->ExecutionUnitsParallelIssue.push_back (-1);
    }
    AccessGranularities.push_back (1);
  }
  
  
  
  for (unsigned i = 0; i < nExecutionUnits; i++)
    ShareThroughputAmongPorts.push_back(false);
  
  if (ConstraintPorts){
    for (unsigned i = 0; i < nExecutionUnits; i++){
      if (i >= nArithmeticExecutionUnits + nMovExecutionUnits
          && i < nArithmeticExecutionUnits + nMovExecutionUnits + nMemExecutionUnits) {
        //  ShareThroughputAmongPorts[i] = true;
        ShareThroughputAmongPorts[i] = false;
        if(ConstraintPortsx86)
          ShareThroughputAmongPorts[i] = true;
        
        
      }
      
    }
    bool ShareMemoryThroughput = false;
    for (unsigned i = nArithmeticExecutionUnits + nMovExecutionUnits ; i< nArithmeticExecutionUnits + nMovExecutionUnits + nMemExecutionUnits; i++){
      if(ShareThroughputAmongPorts[i] == true){
        ShareMemoryThroughput = true;
        break;
        
      }
    }
    if(ShareMemoryThroughput== false){
      size_t MaxParallelIssueLoads = 0;
      size_t MaxParallelIssueStores = 0;
      for (unsigned i = nArithmeticNodes + nMovNodes; i < nArithmeticNodes + nMovNodes + nMemNodes; i++){
        
        
        if (i == L1_LOAD_NODE || i == L2_LOAD_NODE || i == L3_LOAD_NODE || i == MEM_LOAD_NODE ){
        		MaxParallelIssueLoads = max(MaxParallelIssueLoads, DispatchPort[i].size());
          
        }else{
          if (i == L1_STORE_NODE || i == L2_STORE_NODE || i == L3_STORE_NODE || i == MEM_STORE_NODE ){
            MaxParallelIssueStores = max(MaxParallelIssueStores, DispatchPort[i].size());
            
          }
        }
      }
      
      if (ConstraintAGUs && MaxParallelIssueLoads+MaxParallelIssueStores > nAGUs){
        dbgs() << "MaxParallelIssueLoads " << MaxParallelIssueLoads   << "\n";
        dbgs() << "MaxParallelIssueStores " << MaxParallelIssueStores   << "\n";
        dbgs() << "nAGUs " << nAGUs   << "\n";
        report_fatal_error ("Throughput is not shared among memory ports, and maximum parallel issue of loads plus\n maximum paralell issue of stores is larger than the number of available AGUs.\n Therefore, parallel issue is overprovisioned and will never be reached");
      }
    }
  }
  
  // We need AccessWidth and Throughput for every resource for which we calculte
  // span, including ports
  
  // Before the 1 below was nAGUs, but in reality for AGUs there is only one resource
  // with throughput equal to the number of AGUs
  
  for (unsigned i = 0; i < nExecutionUnits + 1 + nPorts + nBuffers; i++) {
    
    unsigned IssueCycleGranularity = 0;
    unsigned AccessWidth = 0;
    
    if (i < nArithmeticExecutionUnits + nMovExecutionUnits) {
      // AccessWidth = VectorWidth;
      AccessWidth = 1;
      // Computational units throughput must also be rounded
      if (i < nArithmeticExecutionUnits) {
        if (this->ExecutionUnitsThroughput[i] != INF) {
          if (this->ExecutionUnitsThroughput[i] >= 1) {
            this->ExecutionUnitsThroughput[i] = roundNextMultiple (this->ExecutionUnitsThroughput[i], VectorWidth);
          }
        }
      }
      else {
        if (this->ExecutionUnitsThroughput[i] != INF) {
          if (this->ExecutionUnitsThroughput[i] >= 1) {
            this->ExecutionUnitsThroughput[i] = this->ExecutionUnitsThroughput[i]; // TODO: ???
          }
        }
        
      }
    }
    else {
      if (i >= nArithmeticExecutionUnits + nMovExecutionUnits
          && i < nArithmeticExecutionUnits + nMovExecutionUnits + nMemExecutionUnits) {
        // AccessWidth = roundNextMultiple(VectorWidth*MemoryWordSize, AccessGranularities[i]);
        AccessWidth = roundNextMultiple (MemoryWordSize, AccessGranularities[i]);
        // Round throughput of memory resources to the next multiple of AccessWidth
        // (before it was MemoryWordSize)
        if (this->ExecutionUnitsThroughput[i] != INF) {
          if (this->ExecutionUnitsThroughput[i] < AccessWidth) {
            // if (this->ExecutionUnitsThroughput[i] < this->MemoryWordSize)
            //HERE  if (this->ExecutionUnitsThroughput[i] < 1)
            if (this->ExecutionUnitsThroughput[i] > 0 && this->ExecutionUnitsThroughput[i] < 1 ) {
              float Inverse = ceil (1 / this->ExecutionUnitsThroughput[i]);
              float Rounded = roundNextPowerOfTwo (Inverse);
              
              if (Inverse == Rounded) {
                
                this->ExecutionUnitsThroughput[i] = float (1) / float (Rounded);
                
              }
              else {
                
                this->ExecutionUnitsThroughput[i] = float (1) / float ((Rounded / float (2)));
              }
            }
            else {
              this->ExecutionUnitsThroughput[i] = roundNextPowerOfTwo (ceil (this->ExecutionUnitsThroughput[i]));
            }
          }
          else {
            // Round to the next multiple of AccessGranularities...
            //          this->ExecutionUnitsThroughput[i] = roundNextMultiple(this->ExecutionUnitsThroughput[i],this->MemoryWordSize);
            this->ExecutionUnitsThroughput[i] =
            roundNextMultiple (this->ExecutionUnitsThroughput[i], AccessGranularities[i]);
          }
        }
      }
      else {
        AccessWidth = 1;
      }
    }
    
    
    
    if (ConstraintPortsx86){
      for (unsigned i = 0; i < nExecutionUnits; i++) {
        if (this->ExecutionUnitsParallelIssue[i] == INF &&  DispatchPort[i].size()!= 0) {
          report_fatal_error ("Parallel Issue cannot be infinity if ConstraintPortsx86");
        }
      }
    }
    

    IssueCycleGranularity = GetIssueCycleGranularity(i,AccessWidth, 1);
    
    AccessWidths.push_back (AccessWidth);
    IssueCycleGranularities.push_back (IssueCycleGranularity);
    
    
    DEBUG (dbgs () << "______________ " << GetResourceName(i) << " __________\n");
    DEBUG (dbgs () << "AccessWidth " << AccessWidth << "\n");
    DEBUG (dbgs () << "ExecutionUnitsParallelIssue " << this->ExecutionUnitsParallelIssue[i] << "\n");
    DEBUG (dbgs () << "ExecutionUnitsThroughput " << this->ExecutionUnitsThroughput[i] << "\n");
    DEBUG (dbgs () << "IssueCycleGranularitiy " << IssueCycleGranularities[i] << "\n");
    
    if(this->ExecutionUnitsLatency[i] != 0 && this->ExecutionUnitsLatency[i] < IssueCycleGranularity ){
      dbgs() << "Resource " << GetResourceName(i) << "\n";
      dbgs() << "Latency " << this->ExecutionUnitsLatency[i] << "\n";
      dbgs() << "IssueCycleGranularities " << IssueCycleGranularity << "\n";
      report_fatal_error ("Latency cannot be smaller than issue cycle granularity");
    }
  }
  
  
  DEBUG (dbgs () << "Number of resources " << nExecutionUnits + nPorts + nAGUs + nLoadAGUs + nStoreAGUs + nBuffers << "\n");
  
  //Some checkings....

  if (LoadBufferSize > 0 && ReservationStationSize == 0)
    report_fatal_error ("RS cannot be zero if LB exists");
  
  if (StoreBufferSize > 0 && ReservationStationSize == 0)
    report_fatal_error ("RS cannot be zero if SB exists");
  
  if (LineFillBufferSize > 0 && LoadBufferSize == 0)
    report_fatal_error ("LB cannot be zero if LFB exists");
  
  //Check that access granularities are either memory word size or cache line size
  if (!MemAccessGranularity.empty()){
  for (unsigned i = 0; i < MemAccessGranularity.size (); i++)
    if (MemAccessGranularity[i] != MemoryWordSize && MemAccessGranularity[i] != CacheLineSize)
      report_fatal_error ("Memory access granularity is not memory word size, nor cache line size\n");
  }else{
	
  }
  if (InOrderExecution && ReorderBufferSize > 0)
    report_fatal_error ("In order execution does not allow to have a ROB of size > 1");
  

  if(L1CacheSize != 0 &&  this->ExecutionUnitsLatency[L2_LOAD_CHANNEL] <= this->ExecutionUnitsLatency[L1_LOAD_CHANNEL])
    report_fatal_error("Latency of L1 cache cannot be larger or equal than latency to other levels");
  if(L2CacheSize != 0 && this-> ExecutionUnitsLatency[L3_LOAD_CHANNEL] <= this->ExecutionUnitsLatency[L2_LOAD_CHANNEL])
    report_fatal_error("Latency of L2 cache cannot be larger or equal than latency to other levels");
   
  if(LLCCacheSize != 0 &&  this->ExecutionUnitsLatency[MEM_LOAD_CHANNEL] <= this->ExecutionUnitsLatency[L3_LOAD_CHANNEL])
    report_fatal_error("Latency of L3 cache cannot be larger or equal than latency to other levels");
  
  

  if (InOrderExecution)
    this->ReorderBufferSize = 1;
  
  // Itinitalize global variables to zero...
  FunctionCallStack = 0;
  TotalInstructions = 0;
  BasicBlockBarrier = 0;
  BasicBlockLookAhead = 0;
  InstructionFetchCycle = 0;
  LoadDispatchCycle = 0;
  StoreDispatchCycle = 0;
  NFirstAccesses = 0;
  LastLoadIssueCycle = 0;
  LastStoreIssueCycle = 0;
  RemainingInstructionsFetch = InstructionFetchBandwidth;
  ReuseTree = NULL;
  PrefetchReuseTree = NULL;
  PrefetchReuseTreeSize = 0;
  LastIssueCycleFinal = 0;
  

  // For resources with throughput and latency, i.e., resources for which we insert
  // cycles
  for (unsigned i = 0; i < nExecutionUnits + nPorts + nAGUs + nLoadAGUs + nStoreAGUs + nBuffers; i++) {
    InstructionsCount.push_back (0);
    InstructionsCountExtended.push_back (0);
    ScalarInstructionsCountExtended.push_back (0);
    VectorInstructionsCountExtended.push_back (0);
    InstructionsSpan.push_back (0);
    InstructionsLastIssueCycle.push_back (0);
    IssueSpan.push_back (0);
    LatencyOnlySpan.push_back (0);
    SpanGaps.push_back (0);
    FirstNonEmptyLevel.push_back (0);
    DAGLevelsOccupancy.push_back (emptyVector);
    MaxOccupancy.push_back (0);
    NInstructionsStalled.push_back (0);
    FirstIssue.push_back (false);
    AverageOverlaps.push_back(0);
    AverageOverlapsCycles.push_back(0);
    OverlapsCount.push_back(0);
    OverlapsDerivatives.push_back(1);
    OverlapsMetrics.push_back(0);
  }
  
  
  
  for (unsigned i = 0; i < nBuffers; i++){
    BuffersOccupancy.push_back (0);
  }
  
  BuffersOccupancyThreshold = 1.0;
  //Initially, FullOccupancyCyclesTree has one element
  //FullOccupancyCyclesTree.push_back(NULL);
#ifdef EFF_TBV
  for (int i = 0; i< MAX_RESOURCE_VALUE; i++)
    FullOccupancyCyclesTree.push_back(TBV_node());
  
#endif
  
  //SplitTreeRange = 65536;
  
  for (unsigned i = 0; i < nExecutionUnits + nPorts + nAGUs + nLoadAGUs + nStoreAGUs + nBuffers; i++)
    AvailableCyclesTree.push_back (NULL);
  
  IssuePorts = vector < unsigned >();
  
  VectorCode = false;
  
  
  CGSFCache.resize (MAX_RESOURCE_VALUE);
  CISFCache.resize (MAX_RESOURCE_VALUE);
  CLSFCache.resize(MAX_RESOURCE_VALUE);
  
  
}



//===----------------------------------------------------------------------===//
//                General functions for the analysis
//===----------------------------------------------------------------------===//

#define HANDLE_MEMORY_TYPE(TY, type, operand)  \
Ty = I.getOperand(operand)->getType();  \
if(PointerType* PT = dyn_cast<PointerType>(Ty)){  \
switch (PT->getElementType()->getTypeID()) {  \
case Type::HalfTyID:  return FP_##TY##_16_BITS; \
case Type::FloatTyID:  return FP_##TY##_32_BITS;  \
case Type::DoubleTyID:  return FP_##TY##_64_BITS; \
case Type::X86_FP80TyID:  return FP_##TY##_80_BITS; \
case Type::PPC_FP128TyID:  return FP_##TY##_128_BITS; \
case Type::X86_MMXTyID:  return FP_##TY##_64_BITS;  \
case Type::VectorTyID: \
switch (PT->getElementType()->getVectorElementType()->getTypeID()) {  \
case Type::HalfTyID:  return FP_##TY##_16_BITS; \
case Type::FloatTyID:  \
return FP_##TY##_32_BITS;  \
case Type::DoubleTyID:  \
return FP_##TY##_64_BITS; \
case Type::IntegerTyID: \
switch (PT->getElementType()->getVectorElementType()->getIntegerBitWidth()) { \
case 4: return INT_##TY##_4_BITS; \
case 8: return INT_##TY##_8_BITS; \
case 16: return INT_##TY##_16_BITS; \
case 32: return INT_##TY##_32_BITS; \
case 64: return INT_##TY##_64_BITS; \
default: return MISC;  \
} \
default: return MISC;\
}\
case Type::IntegerTyID: \
IntegerTy = dyn_cast<IntegerType>(PT-> getElementType());  \
switch (IntegerTy -> getBitWidth()){  \
case 4: return INT_##TY##_4_BITS; \
case 8: return INT_##TY##_8_BITS; \
case 16: return INT_##TY##_16_BITS; \
case 32: return INT_##TY##_32_BITS; \
case 64: return INT_##TY##_64_BITS; \
default: return MISC;  \
} \
default: return MISC; \
} \
}else{  \
errs() << "Trying to load a non-pointer type.\n"; \
} \




string
DynamicAnalysis::GetResourceName (unsigned Resource)
{
  
  switch (Resource) {
    case FP_ADDER:
      return "FP_ADDER";
    case FP_MULTIPLIER:
      return "FP_MULTIPLIER";
    case FP_DIVIDER:
      return "FP_DIVIDER";
    case FP_SHUFFLE_UNIT:
      return "FP_SHUFFLE_UNIT";
    case FP_BLEND_UNIT:
      return "FP_BLEND_UNIT";
    case FP_MOV_UNIT:
      return "FP_MOV_UNIT";
    case REGISTER_LOAD_CHANNEL:
      return "REGISTER_CHANNEL";
    case L1_LOAD_CHANNEL:
      return "L1_LOAD_CHANNEL";
    case L1_STORE_CHANNEL:
      return "L1_STORE_CHANNEL";
    case L2_LOAD_CHANNEL:
      return "L2";
    case L3_LOAD_CHANNEL:
      return "L3 ";
    case MEM_LOAD_CHANNEL:
      return "MEM_LOAD_CHANNEL";
    case ADDRESS_GENERATION_UNIT:
      return "AGU";
    case PORT_0:
      return "PORT_0";
    case PORT_1:
      return "PORT_1";
    case PORT_2:
      return "PORT_2";
    case PORT_3:
      return "PORT_3";
    case PORT_4:
      return "PORT_4";
    case PORT_5:
      return "PORT_5";
      
    case RS_STALL:
      return "RS";
    case ROB_STALL:
      return "ROB";
    case LB_STALL:
      return "LB";
    case SB_STALL:
      return "SB";
    case LFB_STALL:
      return "LFB";
      
    default:
      if (Resource > PORT_5){
        
        char numstr[21]; // enough to hold all numbers up to 64-bits
        sprintf(numstr, "%d", Resource-PORT_0);
        std::string prefix = "PORT_";
        return prefix  + numstr;
      }else{
        return "Resources not known";
      }
  }
  
}



string
DynamicAnalysis::GetNodeName (unsigned Node)
{
  
  switch (Node) {
    case FP_ADD_NODE:
      return "FP_ADD_NODE";
    case FP_MUL_NODE:
      return "FP_MUL_NODE";
    case FP_DIV_NODE:
      return "FP_DIV_NODE";
    case FP_SHUFFLE_NODE:
      return "FP_SHUFFLE_NODE";
    case FP_BLEND_NODE:
      return "FP_BLEND_NODE";
    case FP_MOV_NODE:
      return "FP_MOV_NODE";
    case REGISTER_LOAD_NODE:
      return "REGISTER_LOAD_NODE";
    case REGISTER_STORE_NODE:
      return "REGISTER_STORE_NODE";
    case L1_LOAD_NODE:
      return "L1_LOAD_NODE";
    case L1_STORE_NODE:
      return "L1_STORE_NODE";
    case L2_LOAD_NODE:
      return "L2_LOAD_NODE";
    case L2_STORE_NODE:
      return "L2_STORE_NODE";
    case L3_LOAD_NODE:
      return "L3_LOAD_NODE ";
    case MEM_LOAD_NODE:
      return "MEM_LOAD_NODE ";
    case MEM_STORE_NODE:
      return "MEM_STORE_NODE";
    case AGU_NODE:
      return "AGU_NODE";
    case PORT_0_NODE:
      return "PORT_0_NODE";
    case PORT_1_NODE:
      return "PORT_1_NODE";
    case PORT_2_NODE:
      return "PORT_2_NODE";
    case PORT_3_NODE:
      return "PORT_3_NODE";
    case PORT_4_NODE:
      return "PORT_4_NODE";
    case PORT_5_NODE:
      return "PORT_5_NODE";
    case RS_STALL_NODE:
      return "RS_NODE";
    case ROB_STALL_NODE:
      return "ROB_NODE";
    case LB_STALL_NODE:
      return "LB_NODE";
    case SB_STALL_NODE:
      return "SB_NODE";
    case LFB_STALL_NODE:
      return "LFB_NODE";
      
    default:
      return "Node not known";
      
  }
  
}


// Copy from Instruction.cpp-getOpcodeName()
int
DynamicAnalysis::getInstructionType (Instruction & I)
{
  
  IntegerType *IntegerTy;
  Type *Ty;
  unsigned Opcode = I.getOpcode ();
  
  switch (Opcode) {
      // Terminators
    case Instruction::Ret:
      return CTRL;
    case Instruction::Br:
      return CTRL;
    case Instruction::Switch:
      return CTRL;
    case Instruction::IndirectBr:
      return CTRL;
    case Instruction::Invoke:
      return CTRL;
    case Instruction::Resume:
      return CTRL;
    case Instruction::Unreachable:
      return CTRL;
      
      // Standard binary operators...
    case Instruction::Add:
      return INT_ADD;
    case Instruction::FAdd:
      return FP_ADD;
    case Instruction::Sub:
      return INT_SUB;
    case Instruction::FSub:
      return FP_SUB;
    case Instruction::Mul:
      return INT_MUL;
    case Instruction::FMul:
      return FP_MUL;
    case Instruction::UDiv:
      return INT_DIV;
    case Instruction::SDiv:
      return INT_DIV;
    case Instruction::FDiv:
      return FP_DIV;
    case Instruction::URem:
      return INT_REM;
    case Instruction::SRem:
      return INT_REM;
    case Instruction::FRem:
      return FP_REM;
      
      // Logical operators...
    case Instruction::And:
      return MISC;
    case Instruction::Or:
      return MISC;
    case Instruction::Xor:
      return MISC;
      
      // Memory instructions...
    case Instruction::Alloca:
      return MISC_MEM;
    case Instruction::Load:
      HANDLE_MEMORY_TYPE (LD, Load, 0) case Instruction::Store:
      HANDLE_MEMORY_TYPE (ST, Store, 1);
    case Instruction::AtomicCmpXchg:
      return MISC_MEM;
    case Instruction::AtomicRMW:
      return MISC_MEM;
    case Instruction::Fence:
      return MISC_MEM;
    case Instruction::GetElementPtr:
      return MISC_MEM;
      // Convert instructions...
    case Instruction::Trunc:
      return MISC;
    case Instruction::ZExt:
      return MISC;
    case Instruction::SExt:
      return MISC;
    case Instruction::FPTrunc:
      return MISC;
    case Instruction::FPExt:
      return MISC;
    case Instruction::FPToUI:
      return MISC;
    case Instruction::FPToSI:
      return MISC;
    case Instruction::UIToFP:
      return MISC;
    case Instruction::SIToFP:
      return MISC;
    case Instruction::IntToPtr:
      return MISC;
    case Instruction::PtrToInt:
      return MISC;
    case Instruction::BitCast:
      return MISC;
      
      // Other instructions...
    case Instruction::ICmp:
      return CTRL;
    case Instruction::FCmp:
      return FP_ADD;
    case Instruction::PHI:
      return CTRL;
    case Instruction::Select:
      return CTRL;
    case Instruction::Call:
      return CTRL;
    case Instruction::Shl:
      return MISC;
    case Instruction::LShr:
      return MISC;
    case Instruction::AShr:
      return MISC;
    case Instruction::VAArg:
      return MISC;
    case Instruction::ExtractElement:
      return FP_SHUFFLE;
    case Instruction::InsertElement:
      return FP_SHUFFLE;
    case Instruction::ShuffleVector:
      return FP_SHUFFLE;
    case Instruction::ExtractValue:
      return MISC;
    case Instruction::InsertValue:
      return MISC;
    case Instruction::LandingPad:
      return MISC;
      
    case 60:
      return FP_SHUFFLE;
      
    default:
      return -1;
  }
}




// Copy from Instruction.cpp-getOpcodeName()
int
DynamicAnalysis::getInstructionComputationDAGNode (Instruction & I)
{
  
  unsigned Opcode = I.getOpcode ();
  
  switch (Opcode) {
      // Terminators
    case Instruction::Ret:
      return -1;
    case Instruction::Br:
      return -1;
    case Instruction::Switch:
      return -1;
    case Instruction::IndirectBr:
      return -1;
    case Instruction::Invoke:
      return -1;
    case Instruction::Resume:
      return -1;
    case Instruction::Unreachable:
      return -1;
      
      // Standard binary operators...
    case Instruction::Add:
      return -1;
    case Instruction::FAdd:
      return FP_ADD_NODE;
    case Instruction::Sub:
      return -1;
    case Instruction::FSub:
      return FP_ADD_NODE;
    case Instruction::Mul:
      return -1;
    case Instruction::FMul:
      return FP_MUL_NODE;
    case Instruction::UDiv:
      return -1;
    case Instruction::SDiv:
      return -1;
    case Instruction::FDiv:
      return FP_DIV_NODE;
    case Instruction::URem:
      return -1;
    case Instruction::SRem:
      return -1;
    case Instruction::FRem:
      return FP_REM;
      
      // Logical operators...
    case Instruction::And:
      return -1;
    case Instruction::Or:
      return -1;
    case Instruction::Xor:
      return -1;
      
      // Memory instructions...
    case Instruction::Alloca:
      return -1;
    case Instruction::Load:
      return MEM_LOAD_NODE;
    case Instruction::Store:
      return MEM_LOAD_NODE;
    case Instruction::AtomicCmpXchg:
      return -1;
    case Instruction::AtomicRMW:
      return -1;
    case Instruction::Fence:
      return -1;
    case Instruction::GetElementPtr:
      return -1;
      // Convert instructions...
    case Instruction::Trunc:
      return -1;
    case Instruction::ZExt:
      return -1;
    case Instruction::SExt:
      return -1;
    case Instruction::FPTrunc:
      return -1;
    case Instruction::FPExt:
      return -1;
    case Instruction::FPToUI:
      return -1;
    case Instruction::FPToSI:
      return -1;
    case Instruction::UIToFP:
      return -1;
    case Instruction::SIToFP:
      return -1;
    case Instruction::IntToPtr:
      return -1;
    case Instruction::PtrToInt:
      return -1;
    case Instruction::BitCast:
      return -1;
      
      // Other instructions...
    case Instruction::ICmp:
      return -1;
    case Instruction::FCmp:
      return FP_ADD_NODE;
    case Instruction::PHI:
      return -1;
    case Instruction::Select:
      return -1;
    case Instruction::Call:
      return -1;
    case Instruction::Shl:
      return -1;
    case Instruction::LShr:
      return -1;
    case Instruction::AShr:
      return -1;
    case Instruction::VAArg:
      return -1;
      // case Instruction::ExtractElement: return FP_MOV_NODE;
      // case Instruction::InsertElement:  return FP_MOV_NODE;
    case Instruction::ExtractElement:
      return FP_BLEND_NODE;
    case Instruction::InsertElement:
      return FP_BLEND_NODE;
    case Instruction::ShuffleVector:
      return FP_SHUFFLE_NODE;
    case Instruction::ExtractValue:
      return -1;
    case Instruction::InsertValue:
      return -1;
    case Instruction::LandingPad:
      return -1;
      
    case 60:
      return FP_BLEND_NODE;
      
    default:
      return -1;
  }
}



//===----------------------------------------------------------------------===//
//                  Routines for Analysis of ILP
//===----------------------------------------------------------------------===//

uint64_t
DynamicAnalysis::getInstructionValueIssueCycle (Value * v)
{
  
  uint64_t
  InstructionIssueCycle = 0;
  map < Value *, uint64_t >::iterator IssueCycleMapIt;
  
  
  //&I
  IssueCycleMapIt = InstructionValueIssueCycleMap.find (v);
  if (IssueCycleMapIt != InstructionValueIssueCycleMap.end ()) {
    InstructionIssueCycle = IssueCycleMapIt->second;
    // Reset the value of issue cyle after reading it so that
    // when the next time this instruction is executed, it it not poluted
    // with a previous value. This problems arises when two instances of the
    // same instruction are represented by the same value.
    //  InstructionValueIssueCycleMap.erase(IssueCycleMapIt);
    IssueCycleMapIt->second = 0;
  }
  else
    InstructionIssueCycle = 0;	// First usage
  
  return InstructionIssueCycle;
}



uint64_t
DynamicAnalysis::getInstructionValueInstance (Value * v)
{
  
  uint64_t InstructionInstance = 0;
  map < Value *, uint64_t >::iterator InstanceMapIt;
  
  
  //&I
  InstanceMapIt = InstructionValueInstanceMap.find (v);
  if (InstanceMapIt != InstructionValueInstanceMap.end ()) {

    InstructionInstance = InstanceMapIt->second;

  }else{
    InstructionInstance = 0;	// First usage

     InstructionValueInstanceMap[v] = InstructionInstance ;

  }

  return InstructionInstance;
}



void
DynamicAnalysis::increaseInstructionValueInstance (Value * v)
{
  
  map < Value *, uint64_t >::iterator InstanceMapIt;
  
  //&I
  InstanceMapIt = InstructionValueInstanceMap.find (v);
  if (InstanceMapIt != InstructionValueInstanceMap.end ()) {

    InstanceMapIt->second++;

  }else{

	InstructionValueInstanceMap[v] = 0;


  }

}





//===----------------------------------------------------------------------===//
//        Routines for memory addresses and cache lines
//===----------------------------------------------------------------------===//


CacheLineInfo
DynamicAnalysis::getCacheLineInfo (uint64_t v)
{
  
  CacheLineInfo
  Info;
  uint64_t
  CacheLineIssueCycle = 0;
  uint64_t
  CacheLineLastAccess = 0;
  map < uint64_t, CacheLineInfo >::iterator IssueCycleMapIt;
  
  //&I
  IssueCycleMapIt = CacheLineIssueCycleMap.find (v);
  if (IssueCycleMapIt != CacheLineIssueCycleMap.end ()) {
    CacheLineIssueCycle = IssueCycleMapIt->second.IssueCycle;
    CacheLineLastAccess = IssueCycleMapIt->second.LastAccess;
  }
  else {
    CacheLineIssueCycle = 0;	// First usage
    CacheLineLastAccess = 0;
  }
  
  Info.LastAccess = CacheLineLastAccess;
  Info.IssueCycle = CacheLineIssueCycle;
  // Now this is moved to to getMemoryAddressIssueCycle
  // Info.IssueCycle = max(max(InstructionFetchCycle,BasicBlockBarrier),InstructionIssueCycle);
  
  return (Info);
}



uint64_t
DynamicAnalysis::getCacheLineLastAccess (uint64_t v)
{
  
  uint64_t
  InstructionLastAccess = 0;
  map < uint64_t, CacheLineInfo >::iterator IssueCycleMapIt;
  
  //&I
  IssueCycleMapIt = CacheLineIssueCycleMap.find (v);
  if (IssueCycleMapIt != CacheLineIssueCycleMap.end ())
    InstructionLastAccess = IssueCycleMapIt->second.LastAccess;
  
  return InstructionLastAccess;
}



uint64_t
DynamicAnalysis::getMemoryAddressIssueCycle (uint64_t v)
{
  uint64_t
  IssueCycle = 0;
  map < uint64_t, uint64_t >::iterator IssueCycleMapIt;
  
  //&I
  IssueCycleMapIt = MemoryAddressIssueCycleMap.find (v);
  if (IssueCycleMapIt != MemoryAddressIssueCycleMap.end ())
    IssueCycle = IssueCycleMapIt->second;
  
  return IssueCycle;
}



void
DynamicAnalysis::insertInstructionValueIssueCycle (Value * v, uint64_t InstructionIssueCycle, bool isPHINode)
{
  
  map < Value *, uint64_t >::iterator IssueCycleMapIt;
  
  IssueCycleMapIt = InstructionValueIssueCycleMap.find (v);
  if (IssueCycleMapIt != InstructionValueIssueCycleMap.end ()) {
    if (isPHINode == true)
      IssueCycleMapIt->second = InstructionIssueCycle;
    else{
      IssueCycleMapIt->second = max (IssueCycleMapIt->second, InstructionIssueCycle /*+1 */ );
    }
  }
  else{				//Insert an entry for the instrucion.
    InstructionValueIssueCycleMap[v] = InstructionIssueCycle /*+1 */ ;
   }	
  
}



void
DynamicAnalysis::insertCacheLineInfo (uint64_t v, CacheLineInfo Info)
{
  
  
  map < uint64_t, CacheLineInfo >::iterator IssueCycleMapIt;
  //*i
  IssueCycleMapIt = CacheLineIssueCycleMap.find (v);
  if (IssueCycleMapIt != CacheLineIssueCycleMap.end ()) {
    IssueCycleMapIt->second.IssueCycle = max (IssueCycleMapIt->second.IssueCycle, Info.IssueCycle /*+1 */ );
    IssueCycleMapIt->second.LastAccess = Info.LastAccess;
  }
  else {			//Insert an entry for the instrucion.
    CacheLineIssueCycleMap[v].IssueCycle = Info.IssueCycle /*+1 */ ;
    CacheLineIssueCycleMap[v].LastAccess = Info.LastAccess;
  }
}



void
DynamicAnalysis::insertCacheLineLastAccess (uint64_t v, uint64_t LastAccess)
{
  
  
  map < uint64_t, CacheLineInfo >::iterator IssueCycleMapIt;
  //*i
  IssueCycleMapIt = CacheLineIssueCycleMap.find (v);
  if (IssueCycleMapIt != CacheLineIssueCycleMap.end ()) {
    IssueCycleMapIt->second.LastAccess = LastAccess;
  }
  else				//Insert an entry for the instrucion.
    CacheLineIssueCycleMap[v].LastAccess = LastAccess;
}



void
DynamicAnalysis::insertMemoryAddressIssueCycle (uint64_t v, uint64_t Cycle)
{
  map < uint64_t, uint64_t >::iterator IssueCycleMapIt;
  //*i
  IssueCycleMapIt = MemoryAddressIssueCycleMap.find (v);
  if (IssueCycleMapIt != MemoryAddressIssueCycleMap.end ()) {
    IssueCycleMapIt->second = Cycle;
  }
  else				//Insert an entry for the instrucion.
    MemoryAddressIssueCycleMap[v] = Cycle;
}



//===----------------------------------------------------------------------===//
//        Routines for managing bandwidth and throughput
//===----------------------------------------------------------------------===//


#ifdef EFF_TBV

void DynamicAnalysis::GetTreeChunk (uint64_t i, unsigned int ExecutionResource)
{
  unsigned int size = FullOccupancyCyclesTree[ExecutionResource].BitVector.size();
  if(i >= size){
    FullOccupancyCyclesTree[ExecutionResource].BitVector.resize(i+ SplitTreeRange);
  }
}

#else

uint64_t
DynamicAnalysis::GetTreeChunk (uint64_t i)
{
  
  uint64_t
  TreeChunk = i / SplitTreeRange;
  if (TreeChunk >= FullOccupancyCyclesTree.size ()) {
    FullOccupancyCyclesTree.resize (TreeChunk + 1);
  }
  /*if (TreeChunk >= FullOccupancyCyclesTree.size()) {
   for (unsigned j = FullOccupancyCyclesTree.size(); j<= TreeChunk; j++) {
   FullOccupancyCyclesTree.push_back(NULL);
   }
   } */
  return TreeChunk;
}
#endif

unsigned
DynamicAnalysis::FindNextAvailableIssueCyclePortAndThroughtput (unsigned InstructionIssueCycle,
                                                                unsigned ExtendedInstructionType, unsigned NElementsVector)
{
  
  
  
  
  unsigned ExecutionResource = ExecutionUnit[ExtendedInstructionType];
  unsigned InstructionIssueCycleThroughputAvailable = InstructionIssueCycle;
  uint64_t InstructionIssueCycleFirstTimeAvailable = 0;
  uint64_t InstructionIssueCyclePortAvailable = InstructionIssueCycle;
  uint64_t Port = 0;
  
  
  if(ExecutionUnitsThroughput[ExecutionResource]==0){
    dbgs() << "Throughput zero for resource " << GetResourceName(ExecutionResource) << "\n";
    report_fatal_error("Throughput value not valid for resource issuing instructions");
  }
  
  
  
  bool FoundInThroughput = false;
  bool FoundInPort;
  if (ConstraintPorts) {
    FoundInPort = false;
  }
  else {
    FoundInPort = true;
  }
  
  
  
  while (FoundInThroughput == false || FoundInPort == false) {
    
    if(ConstraintPorts == false)
      InstructionIssueCyclePortAvailable = InstructionIssueCycleThroughputAvailable;
#ifdef DEBUG_GENERIC
    DEBUG (dbgs () << "Starting while\n");
    DEBUG (dbgs () << "FoundInThroughput " << FoundInThroughput << "\n");
    DEBUG (dbgs () << "FoundInPort " << FoundInPort << "\n");
    DEBUG (dbgs () << "Initial potential available issue cycle " << InstructionIssueCycleThroughputAvailable << "\n");
    
    
#endif
    // First, find next available issue cycle based on node throughput
    InstructionIssueCycleThroughputAvailable =
    FindNextAvailableIssueCycle (InstructionIssueCyclePortAvailable, ExecutionResource, NElementsVector);
    DEBUG (dbgs () << "Throughput available in cycle " << InstructionIssueCycleThroughputAvailable << "\n");
    if (InstructionIssueCycleThroughputAvailable == InstructionIssueCyclePortAvailable)
      FoundInThroughput = true;
    
    
    // Check that the port is available
    // Get the ports to which this node binds
    if (ConstraintPorts) {
      bool FirstPortAvailableFound = false;
      
      if(DispatchPort[ExtendedInstructionType].size()== 0 ){
        FoundInPort = true;
      }
      
      for (unsigned i = 0; i < DispatchPort[ExtendedInstructionType].size (); i++) {
        
        // IssuePorts contains ports that have issued instructions in "issuecyclegranularity" cycles before or after current
        // cycle
        bool PortAlreadyDispatch = false;
        for (unsigned j = 0; j< IssuePorts.size();j++){
          if (DispatchPort[ExtendedInstructionType][i] == IssuePorts[j]){
            PortAlreadyDispatch = true;
            break;
          }
          
        }
        if (IssuePorts.size() == 0 || ( PortAlreadyDispatch== false)) {
          
#ifdef DEBUG_GENERIC
          
          DEBUG (dbgs () << "Checking availability in port " << GetResourceName (DispatchPort[ExtendedInstructionType][i]) <<
                 "\n");
#endif
          InstructionIssueCyclePortAvailable =
          FindNextAvailableIssueCycle (InstructionIssueCycleThroughputAvailable, DispatchPort[ExtendedInstructionType][i]);
#ifdef DEBUG_GENERIC
          DEBUG (dbgs () << "Port available in cycle " << InstructionIssueCyclePortAvailable << "\n");
          
#endif
          if (InstructionIssueCyclePortAvailable != InstructionIssueCycleThroughputAvailable) {
            
            if(FirstPortAvailableFound == false){ // First time
              FirstPortAvailableFound = true;
              //            if (i == 0 || IssuePorts.size () != 0)
              InstructionIssueCycleFirstTimeAvailable = InstructionIssueCyclePortAvailable;
            }else {
              InstructionIssueCyclePortAvailable =
              min (InstructionIssueCyclePortAvailable, InstructionIssueCycleFirstTimeAvailable);
              if (InstructionIssueCyclePortAvailable == InstructionIssueCycleFirstTimeAvailable) {
                Port = i;
                if (InstructionIssueCyclePortAvailable == InstructionIssueCycleThroughputAvailable) {
                  FoundInPort = true;
                  break;
                }
              }
            }
          }
          else {
            // If Node is NULL, it is available for sure.
#ifdef DEBUG_GENERIC
            
            DEBUG (dbgs () << "Node is NULL, so port is " << GetResourceName(DispatchPort[ExtendedInstructionType][i]) << "\n");
#endif
            FoundInPort = true;
            Port = i;
            break;
          }
        }
      }
    }
  }
  
  
  //Insert issue cycle in Port and in resource
  if(ConstraintPorts &&  DispatchPort[ExtendedInstructionType].size() != 0)
    InsertNextAvailableIssueCycle (InstructionIssueCyclePortAvailable, DispatchPort[ExtendedInstructionType][Port]);
  
  // Insert in resource!!!
  if (DispatchPort[ExtendedInstructionType].size() != 0)
    InsertNextAvailableIssueCycle (InstructionIssueCyclePortAvailable, ExecutionResource, NElementsVector,
                                   DispatchPort[ExtendedInstructionType][Port]);
  else
    InsertNextAvailableIssueCycle (InstructionIssueCyclePortAvailable, ExecutionResource, NElementsVector);
  
  return InstructionIssueCyclePortAvailable;
}


unsigned
DynamicAnalysis::GetIssueCycleGranularity(unsigned ExecutionResource, unsigned AccessWidth, unsigned NElementsVector){
  
  unsigned IssueCycleGranularity = 1;
  
  /*if(ExecutionUnitsThroughput[ExecutionResource] == INF && ExecutionUnitsParallelIssue[ExecutionResource] == INF)
   IssueCycleGranularity = 1;
   */
  if (ExecutionUnitsThroughput[ExecutionResource] != INF && ExecutionUnitsParallelIssue[ExecutionResource] == INF) {
    IssueCycleGranularity = unsigned (ceil (AccessWidth * NElementsVector/(ExecutionUnitsThroughput[ExecutionResource])));
  }
  
  if (ExecutionUnitsThroughput[ExecutionResource] == INF && ExecutionUnitsParallelIssue[ExecutionResource] != INF) {
    if (ShareThroughputAmongPorts[ExecutionResource]) {
      IssueCycleGranularity = unsigned (ceil (AccessWidth * NElementsVector/(ExecutionUnitsParallelIssue[ExecutionResource])));
    }
  }
  
  if (ExecutionUnitsThroughput[ExecutionResource] != INF && ExecutionUnitsParallelIssue[ExecutionResource] != INF) {
    if (ShareThroughputAmongPorts[ExecutionResource]) {
      IssueCycleGranularity = unsigned (ceil (AccessWidth * NElementsVector/(ExecutionUnitsThroughput[ExecutionResource] *ExecutionUnitsParallelIssue[ExecutionResource])));
    }else
      IssueCycleGranularity = unsigned (ceil (AccessWidth * NElementsVector/(ExecutionUnitsThroughput[ExecutionResource])));
  }
  
  return IssueCycleGranularity;
}



float
DynamicAnalysis::GetEffectiveThroughput(unsigned ExecutionResource, unsigned AccessWidth, unsigned NElementsVector ){
  
  unsigned IssueCycleGranularity = GetIssueCycleGranularity(ExecutionResource, AccessWidth, NElementsVector);
  
  if ( ExecutionUnitsParallelIssue[ExecutionResource] != INF) {
    return (IssueCycleGranularity)* ExecutionUnitsParallelIssue[ExecutionResource];
  }else{
    return INF;
  }
  
}




unsigned
DynamicAnalysis::GetNodeWidthOccupancy(unsigned ExecutionResource, unsigned AccessWidth, unsigned NElementsVector){
  
  unsigned NodeWidthOccupancy =0;
  
  if(ExecutionUnitsThroughput[ExecutionResource] == INF && ExecutionUnitsParallelIssue[ExecutionResource] == INF)
    NodeWidthOccupancy = AccessWidth;
  
  if (ExecutionUnitsThroughput[ExecutionResource] != INF && ExecutionUnitsParallelIssue[ExecutionResource] == INF) {
    if(AccessWidth*NElementsVector <= ExecutionUnitsThroughput[ExecutionResource])
      NodeWidthOccupancy = AccessWidth;
    else
      NodeWidthOccupancy = ceil(ExecutionUnitsThroughput[ExecutionResource]); // AccessWidth before
  }
  
  if (ExecutionUnitsThroughput[ExecutionResource] == INF && ExecutionUnitsParallelIssue[ExecutionResource] != INF) {
    NodeWidthOccupancy = AccessWidth;
  }
  
  if (ExecutionUnitsThroughput[ExecutionResource] != INF && ExecutionUnitsParallelIssue[ExecutionResource] != INF) {
    if (ShareThroughputAmongPorts[ExecutionResource]) {
      if(AccessWidth*NElementsVector <= ExecutionUnitsThroughput[ExecutionResource]* ExecutionUnitsParallelIssue[ExecutionResource])
        NodeWidthOccupancy = AccessWidth*NElementsVector;
      else
        NodeWidthOccupancy = ExecutionUnitsThroughput[ExecutionResource]* ExecutionUnitsParallelIssue[ExecutionResource]; // AccessWidth before
    }else{
      if(AccessWidth*NElementsVector <= ExecutionUnitsThroughput[ExecutionResource])
        NodeWidthOccupancy = AccessWidth;
      else
        NodeWidthOccupancy = ExecutionUnitsThroughput[ExecutionResource]; // AccessWidth before
    }
  }
  
  return NodeWidthOccupancy;
}



bool
DynamicAnalysis::GetLevelFull(unsigned ExecutionResource, unsigned NodeIssueOccupancy, unsigned NodeWidthOccupancy){
  
  bool LevelFull  = false;
  /*
   if(ExecutionUnitsThroughput[ExecutionResource] == INF && ExecutionUnitsParallelIssue[ExecutionResource] == INF)
   LeveFull = false;
   */
  
  // If we can do infinity in parallel (ExecutionUnitsParallelIssue[ExecutionResource] == INF) the level never gets full
  /* if (ExecutionUnitsThroughput[ExecutionResource] != INF && ExecutionUnitsParallelIssue[ExecutionResource] == INF) {
   if(NodeWidthOccupancy >= ExecutionUnitsThroughput[ExecutionResource])
   LevelFull = true;
   
   }*/
  
  if (ExecutionUnitsThroughput[ExecutionResource] == INF && ExecutionUnitsParallelIssue[ExecutionResource] != INF) {
    if(NodeIssueOccupancy == (unsigned)ExecutionUnitsParallelIssue[ExecutionResource])
      LevelFull = true;
  }
  
  if (ExecutionUnitsThroughput[ExecutionResource] != INF && ExecutionUnitsParallelIssue[ExecutionResource] != INF) {
    if (ShareThroughputAmongPorts[ExecutionResource]) {
      if(NodeWidthOccupancy >= ExecutionUnitsThroughput[ExecutionResource]* ExecutionUnitsParallelIssue[ExecutionResource] || NodeIssueOccupancy == (unsigned)ExecutionUnitsParallelIssue[ExecutionResource])
        LevelFull = true;
    }else{
      if(NodeIssueOccupancy == (unsigned)ExecutionUnitsParallelIssue[ExecutionResource])
        LevelFull = true;
    }
  }
  
  return LevelFull;
}




bool
DynamicAnalysis::ThereIsAvailableBandwidth (unsigned NextAvailableCycle, unsigned ExecutionResource,
                                            unsigned NElementsVector, bool & FoundInFullOccupancyCyclesTree,
                                            bool TargetLevel)
{
  
#ifndef EFF_TBV
  unsigned  TmpTreeChunk = 0;
#endif
  bool EnoughBandwidth;
  // float AvailableBandwidth;
  unsigned AccessWidth;
  unsigned IssueCycleGranularity = 0;
  
  Tree < uint64_t > *Node;
  
  // Reset IssuePorts
  IssuePorts = vector < unsigned >();
  
  
  
  if (TargetLevel == true && FoundInFullOccupancyCyclesTree == false) {
    AccessWidth = AccessWidths[ExecutionResource];
    // AvailableBandwidth = ExecutionUnitsThroughput[ExecutionResource];
    
    IssueCycleGranularity = GetIssueCycleGranularity(ExecutionResource, AccessWidth, NElementsVector);
    
    
#ifdef DEBUG_GENERIC
    DEBUG (dbgs () << "Making sure there is also enough bandwidth in cycle " << NextAvailableCycle << "\n");
    DEBUG (dbgs () << "AccessWidth " << AccessWidth << "\n");
    DEBUG (dbgs () << "IssueCycleGranularity " << IssueCycleGranularity << "\n");
    DEBUG (dbgs () << "IssueCycleGranularities[ExecutionResource] " << IssueCycleGranularities[ExecutionResource] << "\n");
    DEBUG (dbgs () << "NElementsVector " << NElementsVector << "\n");
#endif
    // Assume initially that there is enough bandwidth
    EnoughBandwidth = true;
    
    //There is enough bandwidth if:
    // 1. The comp/load/store width fits within the level, or the level is empty.
    
    if (AvailableCyclesTree[ExecutionResource] != NULL) {
      Node = AvailableCyclesTree[ExecutionResource];
      if (Node != NULL && Node->key >= NextAvailableCycle) {
        
        EnoughBandwidth = !GetLevelFull(ExecutionResource,  Node->issueOccupancy, Node->widthOccupancy);
      }
    }
    // Else, if ExecutionUnitsThroughput[ExecutionResource] >= 1, the level is either
    // full (which is not because otherwise this would not being executed), or
    // is available, but still need to check previous and later cycles.
    
    
    // 2. If IssueCycleGranularity > 1, we have to make sure that there were no instructions
    // executed with the same IssueCycleGranularity in previous cycles. We have to do this
    // because we don't include latency cycles in AvailableCyclesTree.
    
    if (EnoughBandwidth == true) {
      
      int64_t StartingCycle = 0;
      
      int64_t tmp = (int64_t) NextAvailableCycle - (int64_t) IssueCycleGranularity + (int64_t) 1;
      
      if (tmp < 0) {
        StartingCycle = 0;
      }
      else
        StartingCycle = NextAvailableCycle - IssueCycleGranularity + 1;
      
#ifdef DEBUG_GENERIC
      DEBUG (dbgs () << "StartingCycle  " << StartingCycle << "\n");
      DEBUG (dbgs () << "NextAvailableCycle  " << NextAvailableCycle << "\n");
#endif
      for (uint64_t i = StartingCycle; i < NextAvailableCycle; i++) {
        
#ifdef EFF_TBV
        GetTreeChunk(i,ExecutionResource);
#else
        TmpTreeChunk = GetTreeChunk (i);
#endif

        
        // If corresponding cycle, for the corresponding resource, is full
#ifdef EFF_TBV
        if (FullOccupancyCyclesTree[ExecutionResource].get_node(i)) {
#else
          if (FullOccupancyCyclesTree[TmpTreeChunk].get_node (i, ExecutionResource)) {
#endif
            FoundInFullOccupancyCyclesTree = true;
            EnoughBandwidth = false;
            //Every time NextAvailableCycle changes, we need to update TreeChunk
            //  TreeChunk = GetTreeChunk(NextAvailableCycle);
            
            // ?? TreeChunk = NextAvailableCycle/SplitTreeRange;
            
#ifdef DEBUG_GENERIC
            DEBUG (dbgs () << "There is not enough bandwidth because of issue cycle granularity in previous cycles\n");
#endif
            break;
#ifdef EFF_TBV
          }
#else
        }
#endif
        else {
          AvailableCyclesTree[ExecutionResource] = splay (i, AvailableCyclesTree[ExecutionResource]);
          Node = AvailableCyclesTree[ExecutionResource];
          if (Node != NULL && Node->key == i) {
            
            if (Node->issuePorts.size() > 0) {
              for (unsigned port = 0; port < Node->issuePorts.size(); port++) {
                
#ifdef DEBUG_GENERIC
                DEBUG (dbgs () << "There was an instruction issued in previous cycle " << i << " in port " << GetResourceName(Node->issuePorts[port]) << "\n");
#endif
                IssuePorts.push_back (Node->issuePorts[port]);
                if (ExecutionUnitsParallelIssue[ExecutionResource] != INF &&  IssuePorts.size() == (unsigned)ExecutionUnitsParallelIssue[ExecutionResource]){
                  EnoughBandwidth = false;
                  
#ifdef DEBUG_GENERIC
                  DEBUG (dbgs () << "There is not enough bandwidth because "<< ExecutionUnitsParallelIssue[ExecutionResource] <<" instructions have been issued in previous cycles (corresponding to issue cycle granularity)\n");
#endif
                  
                  
                }
                
              }
            }
            
          }
        }
      }
      
    }
    // 3. The same as 2 but for next cycles. If there were loads executed on those cycles,
    // there would not be available bandwith for the current load.
    if (EnoughBandwidth == true) {
      
      for (uint64_t i = NextAvailableCycle + 1; i < NextAvailableCycle + IssueCycleGranularity; i++) {
#ifdef DEBUG_GENERIC
        DEBUG (dbgs () << "Checking full occupancy in cycle " << i << "\n");
#endif
        
#ifdef EFF_TBV
        GetTreeChunk (i, ExecutionResource);
#else
        TmpTreeChunk = GetTreeChunk (i);
#endif

#ifdef EFF_TBV
        if(FullOccupancyCyclesTree[ExecutionResource].get_node(i)){
#else
          if (FullOccupancyCyclesTree[TmpTreeChunk].get_node (i, ExecutionResource)) {
#endif
#ifdef DEBUG_GENERIC
            DEBUG (dbgs () << "There is not enough bandwidth because of issue cycle granularity in later cycles\n");
            DEBUG (dbgs () << "Cycle " << i << " is in full\n");
#endif
            FoundInFullOccupancyCyclesTree = true;
            EnoughBandwidth = false;
            
            //  TreeChunk = GetTreeChunk(NextAvailableCycle);
            
#ifdef DEBUG_GENERIC
            DEBUG (dbgs () << "NextAvailableCycle " << NextAvailableCycle << "\n");
#endif
            break;
#ifdef EFF_TBV
          }
#else
        }
#endif
        else {
          AvailableCyclesTree[ExecutionResource] = splay (i, AvailableCyclesTree[ExecutionResource]);
          Node = AvailableCyclesTree[ExecutionResource];
          if (Node != NULL && Node->key == i) {
            if (Node->issuePorts.size () > 0) {
              for (unsigned port = 0; port < Node->issuePorts.size(); port++) {
                
#ifdef DEBUG_GENERIC
                
                DEBUG (dbgs () << "There was an instruction issued in later cycle " << i << " in port " << GetResourceName(Node->issuePorts[port]) << "\n");
#endif
                IssuePorts.push_back (Node->issuePorts[port]);
                
                if (ExecutionUnitsParallelIssue[ExecutionResource] != INF && IssuePorts.size() == (unsigned)ExecutionUnitsParallelIssue[ExecutionResource]){
                  EnoughBandwidth = false;
                  
#ifdef DEBUG_GENERIC
                  DEBUG (dbgs () << "There is not enough bandwidth because "<< ExecutionUnitsParallelIssue[ExecutionResource] <<" instructions have been issued in later cycles (corresponding to issue cycle granularity)\n");
#endif
                  
                  
                }
              }
              
            }
          }
        }
      }
    }
  }
  else {
    EnoughBandwidth = true;
  }
  return EnoughBandwidth;
}



uint64_t
DynamicAnalysis::FindNextAvailableIssueCycleUntilNotInFullOrEnoughBandwidth (unsigned
                                                                             NextCycle, unsigned ExecutionResource,
                                                                             bool & FoundInFullOccupancyCyclesTree,
                                                                             bool & EnoughBandwidth)
{
  
  
  if(ExecutionUnitsThroughput[ExecutionResource]==0){
    dbgs() << "Throughput zero for resource " << GetResourceName(ExecutionResource) << "\n";
    report_fatal_error("Throughput value not valid for resource issuing instructions");
  }
  
  
  
  unsigned NextAvailableCycle = NextCycle;
  unsigned OriginalCycle;
  Tree < uint64_t > *Node = AvailableCyclesTree[ExecutionResource];
  Tree < uint64_t > *LastNodeVisited = NULL;
  
  NextAvailableCycle++;
  
#ifdef DEBUG_GENERIC
  DEBUG (dbgs () << "Searching NextAvailableCycle for " << NextAvailableCycle << "\n");
#endif
  OriginalCycle = NextAvailableCycle;
  
  // If we loop over the first while because there is not enough bandwidth,
  // Node might be NULL because this loop has already been executed.
  Node = AvailableCyclesTree[ExecutionResource];
  
  
#ifdef DEBUG_GENERIC
  DEBUG (dbgs () << "Node->key  " << Node->key << "\n");
  DEBUG (dbgs () << "NextAvailableCycle " << NextAvailableCycle << "\n");
#endif
  while (Node) {
    
    if (Node->key > NextAvailableCycle) {
      if (NextAvailableCycle == OriginalCycle) {	// i.e., it is the first iteration
        NextAvailableCycle = Node->key;
        LastNodeVisited = Node;
        
      }
      // Search for a smaller one
      Node = Node->left;
    }
    else if (Node->key < NextAvailableCycle) {
      //  LastNodeVisited = Node;
      // break;
      // We comment this out because this will never happen in NextAvailable because
      // for every full node we always insert the next available. The general
      // algorithm that finds the larger, if it exist, should have this code
      // uncommented.
      //UNCOMMENT THIS!!
      /*  if (NextAvailableCycle == OriginalCycle){
       NextAvailableCycle = Node->key;
       LastNodeVisited = Node;
       } */
      if (Node->key == OriginalCycle) {
        NextAvailableCycle = OriginalCycle;
        LastNodeVisited = Node;
        
        
        break;
      }
      else if (Node->key > OriginalCycle) {
        //Search for a even smaller one
        NextAvailableCycle = Node->key;
        LastNodeVisited = Node;
        
        // Search for a smaller one
        Node = Node->left;
      }
      else {			//Node->key < OriginalCycle
        // Search for a larger one, but do not store last node visited...
        
        Node = Node->right;
      }
    }
    else {			//Node->key = NextAvailableCycle
      NextAvailableCycle = Node->key;
      LastNodeVisited = Node;
      
      break;
    }
  }
  
  //LastNodeVisited contains the next available cycle. But we still need to check
  //that it is available for lower and upper levels.
  if (LastNodeVisited != NULL && LastNodeVisited->key >= OriginalCycle) {
    NextAvailableCycle = LastNodeVisited->key;
    
  }
  
  //  TreeChunk = GetTreeChunk(NextAvailableCycle);
  
  
  
#ifdef DEBUG_GENERIC
  DEBUG (dbgs () << "NextAvailableCycle " << NextAvailableCycle << "\n");
#endif
  //FoundInFullOccupancyCyclesTree = true;
  FoundInFullOccupancyCyclesTree = false;
  EnoughBandwidth = false;
  return NextAvailableCycle;
  
}

// Find next available issue cycle depending on resource availability.
// Returns a pointer
unsigned
DynamicAnalysis::FindNextAvailableIssueCycle (unsigned OriginalCycle, unsigned ExecutionResource, uint8_t NElementsVector,
                                              bool TargetLevel)
{

  if(ExecutionUnitsThroughput[ExecutionResource]==0){
    dbgs() << "Throughput zero for resource " << GetResourceName(ExecutionResource) << "\n";
    report_fatal_error("Throughput value not valid for resource issuing instructions");
  }
  
  
  uint64_t NextAvailableCycle = OriginalCycle;

  bool FoundInFullOccupancyCyclesTree = true;
  bool EnoughBandwidth = false;
  unsigned TreeChunk = 0;
  // Get the node, if any, corresponding to this issue cycle.
  
#ifdef EFF_TBV
  GetTreeChunk (NextAvailableCycle, ExecutionResource);
#else
  TreeChunk = GetTreeChunk (NextAvailableCycle);
#endif
  
  
  
  // If full is null, then it is available for sure -> WRONG! It might happen that FULL is NULL because
  // a new chunk was created.
  // If it is not NULL and there is something scheduled in this cycle..
  // (we don't include the condition FullOccupancyNode->BitVector[ExecutionResource]==1
  // here because it could happen that it cannot be executed because of throughput<1
  // and something executed in earlier or later cycles.
#ifdef EFF_TBV
  if (!FullOccupancyCyclesTree[ExecutionResource].empty ()) {
#else
    if (!FullOccupancyCyclesTree[TreeChunk].empty ()) {
#endif
      
         // Check if it is in full, but first make sure full is not NULL (it could happen it is NULL after
      // changing the NextAvailableCycle).
#ifdef EFF_TBV
      if(FullOccupancyCyclesTree[ExecutionResource].get_node(NextAvailableCycle)){
        FoundInFullOccupancyCyclesTree = true;
      }
#else
      if (FullOccupancyCyclesTree[TreeChunk].get_node (NextAvailableCycle, ExecutionResource)) {
        FoundInFullOccupancyCyclesTree = true;
      }
#endif
      
      else {
        FoundInFullOccupancyCyclesTree = false;
      }
#ifdef DEBUG_GENERIC
      DEBUG (dbgs () << "Full is not NULL \n");
      DEBUG (dbgs () << "FoundInFullOccupancyCyclesTree " << FoundInFullOccupancyCyclesTree << " \n");
#endif
      // If it is not in full, it is available. But we have to make sure that
      // there is enough bandwidth (to avoid having large trees, we don't include
      // the latency cycles, so we have to make sure we don't issue in in latency cycles)
      if (ExecutionResource < nExecutionUnits) {
        
        if (FoundInFullOccupancyCyclesTree == true) {
          
          NextAvailableCycle =
          FindNextAvailableIssueCycleUntilNotInFullOrEnoughBandwidth (NextAvailableCycle, ExecutionResource,
                                                                      FoundInFullOccupancyCyclesTree, EnoughBandwidth);

          
        }
#ifdef DEBUG_GENERIC
        
        DEBUG (dbgs () << "Searching available BW \n");
#endif
        EnoughBandwidth =
        ThereIsAvailableBandwidth (NextAvailableCycle, ExecutionResource, NElementsVector, FoundInFullOccupancyCyclesTree,
                                   TargetLevel);
        
        if (EnoughBandwidth == false) {
          
          while (EnoughBandwidth == false) {
#ifdef DEBUG_GENERIC
            
            DEBUG (dbgs () << "FindNextAvailableIssueCycleUntilNotInFullOrEnoughBandwidth for next available cycle " <<
                   NextAvailableCycle << " \n");
#endif
            NextAvailableCycle =
            FindNextAvailableIssueCycleUntilNotInFullOrEnoughBandwidth (NextAvailableCycle, ExecutionResource,
                                                                        FoundInFullOccupancyCyclesTree, EnoughBandwidth);
            EnoughBandwidth =
            ThereIsAvailableBandwidth (NextAvailableCycle, ExecutionResource, NElementsVector,
                                       FoundInFullOccupancyCyclesTree, TargetLevel);
            
            // NextAvailableCycle has changed, possibly moving to a different chunk
#ifdef EFF_TBV
            GetTreeChunk (NextAvailableCycle, ExecutionResource);
#else
            TreeChunk = GetTreeChunk (NextAvailableCycle);
#endif
            
          }
          
        }
        // }
        
        
      }
      else {
        if (FoundInFullOccupancyCyclesTree == true) {
          
          while (FoundInFullOccupancyCyclesTree) {
            //FullOccupancyCyclesTree[TreeChunk] = splay(NextAvailableCycle, FullOccupancyCyclesTree[TreeChunk]);
            //Check if it is in full
#ifdef EFF_TBV
            if(FullOccupancyCyclesTree[ExecutionResource].get_node(NextAvailableCycle)){
              //  if (FullOccupancyCyclesTree[ExecutionResource].get_node (TreeChunk, NextAvailableCycle % SplitTreeRange)) {
#else
              if (FullOccupancyCyclesTree[TreeChunk].get_node (NextAvailableCycle, ExecutionResource)) {
#endif
#ifdef DEBUG_GENERIC
                DEBUG (dbgs () << "Cycle " << NextAvailableCycle << " found in Full OccupancyCyclesTree\n");
#endif
                // Try next cycle
                NextAvailableCycle++;
#ifdef EFF_TBV
                GetTreeChunk (NextAvailableCycle, ExecutionResource);
#else
                TreeChunk = GetTreeChunk (NextAvailableCycle);
#endif
                
                
                
                FoundInFullOccupancyCyclesTree = true;
#ifdef EFF_TBV
              }
#else
            }
#endif
            else {
#ifdef DEBUG_GENERIC
              DEBUG (dbgs () << "Cycle " << NextAvailableCycle << " not found in Full OccupancyCyclesTree\n");
#endif
              FoundInFullOccupancyCyclesTree = false;
            }
          }
        }
      }
      
      //  }
#ifdef EFF_TBV
    }
#else
  }
#endif
  else { // If FullOccupancyCyclesTree is empty
    
    if (TreeChunk != 0) {
      
      // Full is NULL, but check that TreeChunk is not zero. Otherwise, Full is not really NULL
      if (ExecutionResource < nExecutionUnits) {
        
        FoundInFullOccupancyCyclesTree = false;
        EnoughBandwidth =
        ThereIsAvailableBandwidth (NextAvailableCycle, ExecutionResource, NElementsVector, FoundInFullOccupancyCyclesTree,
                                   TargetLevel);
        
        if (FoundInFullOccupancyCyclesTree == true || EnoughBandwidth == false) {
          
          NextAvailableCycle =
          FindNextAvailableIssueCycleUntilNotInFullOrEnoughBandwidth (NextAvailableCycle, ExecutionResource,
                                                                      FoundInFullOccupancyCyclesTree, EnoughBandwidth);
        }
      }
    }
    
  }
  
  
  return NextAvailableCycle;
}



// Find next available issue cycle depending on resource availability
bool
DynamicAnalysis::InsertNextAvailableIssueCycle (uint64_t NextAvailableCycle, unsigned ExecutionResource,
                                                unsigned NElementsVector, int IssuePort, bool isPrefetch)
{
  
  Tree < uint64_t > *Node = AvailableCyclesTree[ExecutionResource];
  unsigned NodeIssueOccupancy = 0;
  unsigned NodeWidthOccupancy = 0;
  unsigned NodeOccupancyPrefetch = 0;
  bool LevelGotFull = false;
  
#ifdef EFF_TBV
  GetTreeChunk (NextAvailableCycle, ExecutionResource);
#else
  unsigned TreeChunk = GetTreeChunk (NextAvailableCycle);
#endif
  
  
  // Update Instruction count
  if (InstructionsCountExtended[ExecutionResource] == 0) {
    FirstIssue[ExecutionResource] = true;
  }
  
  if (NElementsVector > 1) {
#ifdef DEBUG_GENERIC
    
    DEBUG (dbgs () << "Increasing instruction count of resource " << GetResourceName (ExecutionResource) << " by " <<
           NElementsVector << "\n");
#endif
    InstructionsCountExtended[ExecutionResource] = InstructionsCountExtended[ExecutionResource] + NElementsVector;
    VectorInstructionsCountExtended[ExecutionResource]++;
    
  }
  else {
    InstructionsCountExtended[ExecutionResource]++;
    ScalarInstructionsCountExtended[ExecutionResource]++;
    
  }

  unsigned AccessWidth = AccessWidths[ExecutionResource];
#ifdef DEBUG_GENERIC
  
  DEBUG (dbgs () << "AccessWidth " << AccessWidth << "\n");
  DEBUG (dbgs () << "NElementsVector " << NElementsVector << "\n");
#endif
  
  if (FirstIssue[ExecutionResource] == true) {
    FirstNonEmptyLevel[ExecutionResource] = NextAvailableCycle;
    FirstIssue[ExecutionResource] = false;
  }
  else {
    FirstNonEmptyLevel[ExecutionResource] = min (FirstNonEmptyLevel[ExecutionResource], NextAvailableCycle);
  }
  
  InstructionsLastIssueCycle[ExecutionResource] = max (InstructionsLastIssueCycle[ExecutionResource], NextAvailableCycle);
  
  
  
#ifdef DEBUG_GENERIC
  DEBUG (dbgs () <<
         "Updating InstructionsLastIssueCycle of execution resource " << GetResourceName (ExecutionResource) << " to " <<
         InstructionsLastIssueCycle[ExecutionResource] << "\n");
  DEBUG (dbgs () << "Inserting next available issue cycle " << NextAvailableCycle << " in execution unit " <<
         GetResourceName (ExecutionResource) << "\n");
  
#endif
  // Insert
  // If it exists already in Available... Inserting it has any effect? No, it simply returns a pointer to the node.
  // Here, use ExtendedInstructionType and not InstructioTypeStats because both prefetched loads and normal loads
  // share the same tree
  // TODO: IF we know in advanced that the available level gets full directly, we can avoid inserting it and removing it
  // from AvailableCyclesTree.
  
  
  AvailableCyclesTree[ExecutionResource] = insert_node (NextAvailableCycle, AvailableCyclesTree[ExecutionResource]);
#ifdef SOURCE_CODE_ANALYSIS
  AvailableCyclesTree[ExecutionResource]->SourceCodeLinesOperationPair.
  push_back (std::make_pair (SourceCodeLine, ExecutionResource));
  //TODO!
  FullOccupancyCyclesTree[TreeChunk].insert_source_code_line (NextAvailableCycle, SourceCodeLine, ExecutionResource);
#endif
  if (IssuePort >= PORT_0)
    AvailableCyclesTree[ExecutionResource]->issuePorts.push_back (IssuePort);
  Node = AvailableCyclesTree[ExecutionResource];
  
  
  
  // If ExecutionUnitsThroughput[ExecutionResource] < 1 , access width is always going to be smaller
  if (isPrefetch) {
    Node->occupancyPrefetch++;
  }
  else {
    Node->issueOccupancy++;
  }
#ifdef DEBUG_GENERIC
  
  DEBUG (dbgs () << "AccessWidth*NElementsVector " << AccessWidth * NElementsVector << "\n");
  DEBUG (dbgs () << "ExecutionUnitsThroughput[ExecutionResource] " << ExecutionUnitsThroughput[ExecutionResource] << "\n");
  DEBUG (dbgs () << "ExecutionUnitsParallelIssue[ExecutionResource] " << ExecutionUnitsParallelIssue[ExecutionResource] <<
         "\n");
#endif

  
  // New, with all the clean cases
  Node->widthOccupancy += GetNodeWidthOccupancy(ExecutionResource, AccessWidth, NElementsVector);
  
  /* Copy these values becasue later on the Node is not the same anymore */
  NodeIssueOccupancy = Node->issueOccupancy;
  NodeWidthOccupancy = Node->widthOccupancy;
  NodeOccupancyPrefetch = Node->occupancyPrefetch;
  MaxOccupancy[ExecutionResource] = max (MaxOccupancy[ExecutionResource], NodeIssueOccupancy + NodeOccupancyPrefetch);
#ifdef DEBUG_GENERIC
  
  DEBUG (dbgs () << "NodeWidthOccupancy " << NodeWidthOccupancy << "\n");
  DEBUG (dbgs () << "NodeIssueOccupancy " << NodeIssueOccupancy << "\n");
#endif
  // If ExecutionUnitsThroughput and ExecutionUnitsParallelIssue are INF, the level never gets full.
  // Otherwise, a level gets full then:
  // 1. The number of parallel operations issued is equal to ExecutionUnitParallelIssue, although the full
  //    bw is not utilized
  // 2. When width occupacy equals to Throughput*ParallelIssue
  

  LevelGotFull = GetLevelFull(ExecutionResource, NodeIssueOccupancy, NodeWidthOccupancy);
  
  if (LevelGotFull) {
    
    LevelGotFull = true;
    
    // Check whether next cycle is in full. because if it is, it should not be inserted into AvailableCyclesTree
    // Next cycle is not NexAvailableCycle+1, is NextAvailableCycle + 1/Throughput
    // Here is where the distinction betweeen execution resource and instruction type is important.
    //  unsigned NextCycle = CalculateIssueCycleGranularity(ExecutionResource, NElementsVector);
    unsigned NextCycle = GetIssueCycleGranularity(ExecutionResource, AccessWidth, NElementsVector);
    
    AvailableCyclesTree[ExecutionResource] = delete_node (NextAvailableCycle, AvailableCyclesTree[ExecutionResource]);
    
    // Insert node in FullOccupancy
    //    FullOccupancyCyclesTree = insert_node(NextAvailableCycle,)
#ifdef DEBUG_GENERIC
    DEBUG (dbgs () << "Level got full\n");
    DEBUG (dbgs () << "AccessWidth " << AccessWidth << "\n");
    DEBUG (dbgs () << "Next Cycle " << NextCycle << "\n");
    DEBUG (dbgs () << "Inserting in FullOccupancyCyclesTree of type " << GetResourceName (ExecutionResource) <<
           " node with key " << NextAvailableCycle << "\n");
#endif
    
#ifdef EFF_TBV
    FullOccupancyCyclesTree[ExecutionResource].insert_node ( NextAvailableCycle);
#else
    FullOccupancyCyclesTree[TreeChunk].insert_node (NextAvailableCycle, ExecutionResource);
#endif
#ifdef SOURCE_CODE_ANALYSIS
#ifdef DEBUG_SOURCE_CODE_ANALYSIS
    DEBUG (dbgs () << "Inserting source code line " << SourceCodeLine <<
           " for cycle " << NextAvailableCycle << " in FullOccupancyTree due to resource " <<
           GetResourceName (ExecutionResource) << "\n");
#endif
    
    
    //FullOccupancyCyclesTree[TreeChunk]->SourceCodeLines.insert(NextAvailableCycle,SourceCodeLine);
    
    FullOccupancyCyclesTree[TreeChunk].insert_source_code_line (NextAvailableCycle, SourceCodeLine, ExecutionResource);
#endif
#ifdef DEBUG_GENERIC
    DEBUG (dbgs () << "ExecutionResource = " << GetResourceName (ExecutionResource) << "\n");
    
    // We don't need to do this anymore because it will be larger than LastIssueCycle - TODO
    // Do it only if NextAvailableCycle+1 == InstructionsLastIssueCycle
    // Insert if it is not in full
    DEBUG (dbgs () << "Checking if key " << NextAvailableCycle + NextCycle << " is in Full Occupancy Tree\n");
#endif
    
#ifdef EFF_TBV
    GetTreeChunk (NextAvailableCycle + NextCycle, ExecutionResource);
#else
    TreeChunk = GetTreeChunk (NextAvailableCycle + NextCycle);
#endif
    
    //  TreeChunk = GetTreeChunk (NextAvailableCycle + NextCycle);
    
#ifdef DEBUG_GENERIC
    DEBUG (dbgs () << "FullOccupancyCyclesTree.size() " << FullOccupancyCyclesTree.size () << "\n");
    
#endif
    // Instead of splay, we need insert_node
    //FullOccupancyCyclesTree[TreeChunk] = splay(NextAvailableCycle+NextCycle/*1*/,  FullOccupancyCyclesTree[TreeChunk]);
    
#ifdef EFF_TBV
    if (	 !FullOccupancyCyclesTree[ExecutionResource].get_node((NextAvailableCycle + NextCycle))) {
#else
      if (			//FullOccupancyCyclesTree[TreeChunk].empty() || // get_node also checks for empty
          !FullOccupancyCyclesTree[TreeChunk].get_node ((NextAvailableCycle + NextCycle), ExecutionResource)) {
#endif
#ifdef DEBUG_GENERIC
        DEBUG (dbgs () << "The next node was not in full, so insert in available " << NextAvailableCycle + NextCycle << "\n");
#endif
        AvailableCyclesTree[ExecutionResource] =
        insert_node (NextAvailableCycle + NextCycle, AvailableCyclesTree[ExecutionResource]);
        
        // In this case, althought we are inserting a node into AvailableCycles, we don't insert the source
        // code line associated to the cycle beacuse it does not mean that an instruction has actually been
        // scheduled in NextAvailableCycle+NextCycle. In this case it just means that this is the next
        // available cycle. Actually, IssueOccupacy of this new level should be zero.
        
        //Update LastIssueCycle -> Distinguish prefetch loads/stores!!
        InstructionsLastIssueCycle[ExecutionResource] =
        max (InstructionsLastIssueCycle[ExecutionResource], NextAvailableCycle + NextCycle);
#ifdef DEBUG_GENERIC
        DEBUG (dbgs () <<
               "Updating InstructionsLastIssueCycle of execution resource " << GetResourceName (ExecutionResource) << " to " <<
               InstructionsLastIssueCycle[ExecutionResource] << "\n");
#endif
#ifdef EFF_TBV
      }
#else
    }
#endif
    
  }

  return LevelGotFull;
}



//===----------------------------------------------------------------------===//
//                Routines for Analysis of Data Reuse
// From the paper "Program Locality Analysis Using Reuse Distance", by Y. Zhong,
// X. Sheng and C. DIng, 2009
//===----------------------------------------------------------------------===//



int
DynamicAnalysis::RegisterStackReuseDistance(uint64_t address){
  
  int Distance = -1;
  std::deque<uint64_t>::iterator itPosition;
  unsigned PositionCounter = 0;
  
  if(RegisterFileSize == 0)
    return Distance;
  
  for (std::deque<uint64_t>::iterator it = ReuseStack.end()-1; it>= ReuseStack.begin(); --it){
    
    if (*it == address){
      Distance = PositionCounter;
      itPosition = it;
      break;
    }
    
    PositionCounter++;
  }
  // If element was not in ReuseStack,
  // - If size of ReuseStack is equal to RegisterFile, pop front and push back the new element
  // - Else, simply push_back
  if (Distance < 0){
    if(ReuseStack.size() == RegisterFileSize){
      ReuseStack.pop_front();
      ReuseStack.push_back(address);
    }else{
      ReuseStack.push_back(address);
    }
    
  }else{
    // Remove and put at the top (back) of the stack
    ReuseStack.erase(itPosition);
    ReuseStack.push_back(address);
  }
  
#ifdef DEBUG_REGISTER_FILE
  DEBUG(dbgs()<<"Memory accesses in register file size:\n");
  for (std::deque<uint64_t>::iterator it = ReuseStack.begin(); it< ReuseStack.end(); ++it){
    DEBUG(dbgs() << *it <<"\t");
  }
  DEBUG(dbgs()<<"\n");
#endif
  return Distance;
}

int
DynamicAnalysis::ReuseDistance (uint64_t Last, uint64_t Current, uint64_t address, bool FromPrefetchReuseTree)
{
  
  int Distance = -1;
  int PrefetchReuseTreeDistance = 0;
  if (!(L1CacheSize == 0 && L2CacheSize == 0 && LLCCacheSize == 0) ) {	// Otherwise, does not matter the distance, it is mem access
    
    int ReuseTreeDistance = ReuseTreeSearchDelete (Last, address, false);
    
    if (SpatialPrefetcher == true) {
      bool IsInPrefetchReuseTree = false;
      // To know whether the data item was in PrefetchReuseTree or not, we check whether the
      // element has been removed from the tree (i.e., the size of the tree).
      //int PrefetchReuseTreeSizeBefore = tree_size(PrefetchReuseTree);
      int PrefetchReuseTreeSizeBefore = PrefetchReuseTreeSize;
      PrefetchReuseTreeDistance = ReuseTreeSearchDelete (Last, address, true);
      
      
      // int PrefetchReuseTreeSizeAfter = tree_size(PrefetchReuseTree);
      int PrefetchReuseTreeSizeAfter = PrefetchReuseTreeSize;
      
      if (PrefetchReuseTreeSizeAfter < PrefetchReuseTreeSizeBefore) {
        IsInPrefetchReuseTree = true;
      }
      else {
        DEBUG (dbgs () << "This data item has not been prefetched\n");
      }
      if (IsInPrefetchReuseTree == false) {
        if (ReuseTreeDistance > 0 && (uint64_t) ReuseTreeDistance >= PrefetchLevel) {
          if (PrefetchReuseTreeDistance >= 0) {
            Distance = ReuseTreeDistance + PrefetchReuseTreeDistance;
          }
          else
            Distance = ReuseTreeDistance;	// In case PrefetchReuseTreeDistance returns -1
        }
        else {
          // If is not a prefetched node and ReuseTreeDistance = -1 or is
          // not a prefetched node and ReuseDistance < L2CacheSize
          Distance = ReuseTreeDistance;
        }
      }
      else {
        DEBUG (dbgs () << "The element is in prefetch tree\n");
        
        // If the data item is a prefetched data item (found in PrefetchReuseTree)
        Distance = PrefetchLevel;
        //   if (!(WarmCache && rep == 0))
        DEBUG (dbgs () << "Distance " << Distance << "\n");
        if (ReuseTreeDistance >= 0) {
          Distance += ReuseTreeDistance;
          //  if (!(WarmCache && rep == 0))
          DEBUG (dbgs () << "Increasing distance to  " << Distance << "\n");
        }
        else {
          // The data item has only been prefetched. In that case, the distance is
          // the size of L2 plus the data items prefetched since the last access
        }
        if (PrefetchReuseTreeDistance >= 0) {
          Distance += PrefetchReuseTreeDistance;
          DEBUG (dbgs () << "Increasing distance to  " << Distance << "\n");
          
        }
        else {
          report_fatal_error
          ("The data item is prefetched, PrefetchReuseTreeDistance >=0, but data item does not seem to be in PrefetchReuseTree");
        }
      }
      
      // In the rest of the cases (ReuseTreeDistance > 0 && ReuseTreeDistance)
    }
    else
      Distance = ReuseTreeDistance;
    
#ifdef ROUND_REUSE_DISTANCE
    if (Distance >= 0)
      Distance = roundNextPowerOfTwo (Distance);
#endif
    
    // Get a pointer to the resulting tree
    if (FromPrefetchReuseTree == false) {
      ReuseTree = insert_node (Current, ReuseTree, address);
      
    }
    else {
      PrefetchReuseTree = insert_node (Current, PrefetchReuseTree, address);
      PrefetchReuseTreeSize++;
    }
    
    
  }
  else {
    
    ReuseTree = insert_node (address, ReuseTree, address);
  }
  
  return Distance;
  
}



// Return the distance of the closest node with key <= Current, i.e., all the
// nodes that have been prefetched between Last and Current.

int
DynamicAnalysis::ReuseTreeSearchDelete (uint64_t Original, uint64_t address, bool FromPrefetchReuseTree)
{
  
  Tree < uint64_t > *Node = NULL;
  int Distance = 0;
  
  if (FromPrefetchReuseTree == false) {
    Node = ReuseTree;
  }
  else
    Node = PrefetchReuseTree;
  
  //Once we find it, calculate the distance without deleting the node.
  
  if (Original == 0 || Node == NULL) {	// Did not find any node smaller
    Distance = -1;
  }
  else {
    
    while (true) {
      // This is the mechanism used in the original algorithm to delete the host
      // node,  decrementing the last_record attribute of the host node, and
      // Node->size = Node->size-1;
      if (Original < Node->key) {
        if (Node->right != NULL)
          Distance = Distance + Node->right->size;
        if (Node->left == NULL)
          break;
        
        Distance = Distance + 1 /*Node->last_record */ ;
        Node = Node->left;
      }
      else {
        if (Original > Node->key) {
          if (Node->right == NULL)
            break;
          Node = Node->right;
        }
        else {			// Last = Node->key, i.e., Node is the host node
          if (Node->right != NULL)
            Distance = Distance + Node->right->size;
          
          //increase by one so that we can calculate directly the hit rate
          // for a cache size multiple of powers of two.
          
          Distance = Distance + 1;
          
          if (Node->address == address && FromPrefetchReuseTree == false) {
            ReuseTree = delete_node (Original, ReuseTree);
            
          }
          else {
            if (Node->address == address && FromPrefetchReuseTree == true) {
              PrefetchReuseTree = delete_node (Original, PrefetchReuseTree);
              PrefetchReuseTreeSize--;
              
            }
          }
          break;
        }
      }
    }
  }
  
  return Distance;
}



void
DynamicAnalysis::updateReuseDistanceDistribution (int Distance, uint64_t InstructionIssueCycle)
{
  
  map < int, int >::iterator ReuseDistanceMapIt;
  map < int, map < uint64_t, uint > >::iterator ReuseDistanceExtendedMapIt;
  map < uint64_t, uint >::iterator AuxMapIt;
  
#ifdef NORMAL_REUSE_DISTRIBUTION
  ReuseDistanceMapIt = ReuseDistanceDistribution.find (Distance);
  if (ReuseDistanceMapIt != ReuseDistanceDistribution.end ()) {
    ReuseDistanceMapIt->second = ReuseDistanceMapIt->second + 1;
  }
  else
    ReuseDistanceDistribution[Distance] = 1;	// First usage
#else
  ReuseDistanceExtendedMapIt = ReuseDistanceDistributionExtended.find (Distance);
  if (ReuseDistanceExtendedMapIt != ReuseDistanceDistributionExtended.end ()) {
    AuxMapIt = (ReuseDistanceExtendedMapIt->second).find (InstructionIssueCycle);
    if (AuxMapIt != (ReuseDistanceExtendedMapIt->second).end ()) {
      AuxMapIt->second = AuxMapIt->second + 1;
    }
    else
      (ReuseDistanceExtendedMapIt->second)[InstructionIssueCycle] = 1;
  }
  else
    ReuseDistanceDistributionExtended[Distance][InstructionIssueCycle] = 1;	// First usage
#endif
}


//===----------------------------------------------------------------------===//
//                  Some auxiliary routines
//===----------------------------------------------------------------------===//




unsigned int
DynamicAnalysis::DivisionRoundUp (float a, float b)
{
  return (a * b + (a + b) / 2) / (a + b);
}



// compute the next highest power of 2 of 32-bit v.
// Routine from Bit Twiddling Hacks, University of Standford.
unsigned int
DynamicAnalysis::roundNextPowerOfTwo (unsigned int v)
{
  
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  v++;
  
  return v;
}



unsigned int
DynamicAnalysis::roundNextMultiple (uint64_t num, int factor)
{
  // This works because factor is always going to be a power of 2
  return (num + factor - 1) & ~(factor - 1);
}



unsigned
DynamicAnalysis::GetMemoryInstructionType (int ReuseDistance, uint64_t MemoryAddress, bool isLoad)
{
  
  if (ReuseDistance < 0) {
    if (L1CacheSize == 0) {
      if (isLoad == true)
        return L1_LOAD_NODE;
      else
        return L1_STORE_NODE;
    }
    else {
      return MEM_LOAD_NODE;
    }
  }
  
  
  if (ReuseDistance <= (int) RegisterFileSize) {
    return REGISTER_LOAD_NODE;
  }
  
  if ((int) RegisterFileSize < ReuseDistance && ReuseDistance <= (int) L1CacheSize){
    if (isLoad == true)
      return L1_LOAD_NODE;
    else
      return L1_STORE_NODE;
  }
  
  if ((int) L1CacheSize < ReuseDistance && ReuseDistance <= (int) L2CacheSize)
    return L2_LOAD_NODE;
  
  if ((int) L2CacheSize < ReuseDistance && ReuseDistance <= (int) LLCCacheSize)
    return L3_LOAD_NODE;
  
  return MEM_LOAD_NODE;
  
}

unsigned
DynamicAnalysis::GetPositionSourceCodeLineInfoVector (uint64_t Resource)
{
  switch (Resource) {
    case FP_ADDER:
      return 0;
      break;
    case FP_MULTIPLIER:
      return 2;
      break;
    case FP_DIVIDER:
      return 4;
      break;
    case FP_SHUFFLE_UNIT:
      return 6;
      break;
    case FP_BLEND_UNIT:
      return 8;
      break;
    case REGISTER_LOAD_CHANNEL:
      return 10;
      break;
      
    case L1_LOAD_CHANNEL:
      return 12;
      break;
    case L1_STORE_CHANNEL:
      return 14;
      break;
    case L2_LOAD_CHANNEL:
      return 16;
      break;
      
    case L3_LOAD_CHANNEL:
      return 18;
      break;
      
    case MEM_LOAD_CHANNEL:
      return 20;
      break;
      
    case RS_STALL:
      return 22;
      break;
    case ROB_STALL:
      return 23;
      break;
    case LB_STALL:
      return 24;
      break;
    case SB_STALL:
      return 25;
      break;
    case LFB_STALL:
      return 26;
      break;
    default:
      dbgs () << "Resource: " << GetResourceName (Resource) << "\n";
      report_fatal_error ("Unknown resource while retrieving source code line information.");
      break;
  }
}

unsigned
DynamicAnalysis::GetExtendedInstructionType (int OpCode, int ReuseDistance, int RegisterStackReuseDistance)
{
  
  unsigned InstructionType = 0;
  
  
  
  switch (OpCode) {
    case Instruction::Add:
#ifdef INT_FP_OPS
      return INT_ADD_NODE;
#else
      report_fatal_error ("Instruction type not associated with a node");
#endif
      
    case Instruction::Sub:
#ifdef INT_FP_OPS
      return INT_ADD_NODE;
#else
      report_fatal_error ("Instruction type not associated with a node");
#endif
    case Instruction::Mul:
#ifdef INT_FP_OPS
      return INT_MUL_NODE;
#else
      report_fatal_error ("Instruction type not associated with a node");
#endif
    case Instruction::UDiv:
    case Instruction::SDiv:
#ifdef INT_FP_OPS
      return INT_DIV_NODE;
#else
      report_fatal_error ("Instruction type not associated with a node");
#endif
    case Instruction::FAdd:
      return FP_ADD_NODE;
      
    case Instruction::FSub:
      return FP_ADD_NODE;
      
    case Instruction::FCmp:
      return FP_ADD_NODE;
      
    case Instruction::FMul:
      return FP_MUL_NODE;
      
    case Instruction::FDiv:
      return FP_DIV_NODE;
      
    case Instruction::InsertElement:
    case Instruction::ExtractElement:
      //return FP_MOV_NODE;
      return FP_BLEND_NODE;
    case Instruction::ShuffleVector:
      return FP_SHUFFLE_NODE;
      
    case Instruction::Load:
      
      if(RegisterStackReuseDistance >= 0)
        return REGISTER_LOAD_NODE;
      else{
        // If ReuseDistance < 0, it is always MEM_LOAD_NODE
        // if (ReuseDistance < 0 && L1CacheSize == 0)
        // return L1_LOAD_NODE;
        
        if (ReuseDistance < 0 || (ReuseDistance > (int) LLCCacheSize && LLCCacheSize != 0))
          return MEM_LOAD_NODE;
        
        
        if (ReuseDistance <= (int) L1CacheSize)
          return L1_LOAD_NODE;
        
        
        if ((int) L1CacheSize < ReuseDistance && (ReuseDistance <= (int) L2CacheSize && L2CacheSize != 0))
          return L2_LOAD_NODE;
        if ((int) L2CacheSize < ReuseDistance && (ReuseDistance <= (int) LLCCacheSize && LLCCacheSize != 0))
          return L3_LOAD_NODE;
      }
      report_fatal_error ("Instruction type not associated with a node");
      break;
      
    case Instruction::Store:
      
      if(RegisterStackReuseDistance >= 0)
        return REGISTER_STORE_NODE;
      else{
        
        if (ReuseDistance < 0 && L1CacheSize == 0)
          return L1_STORE_NODE;
        
        if (ReuseDistance < 0 || (ReuseDistance > (int) LLCCacheSize && LLCCacheSize != 0))
          return MEM_STORE_NODE;
        
        if (ReuseDistance <= (int) L1CacheSize)
          return L1_STORE_NODE;
        
        
        
        
        if ((int) L1CacheSize < ReuseDistance && (ReuseDistance <= (int) L2CacheSize && L2CacheSize != 0))
          return L2_STORE_NODE;
        
        if ((int) L2CacheSize < ReuseDistance && (ReuseDistance <= (int) LLCCacheSize && LLCCacheSize != 0))
          return L3_STORE_NODE;
      }
      report_fatal_error ("Instruction type not associated with a node");
      break;
      
    case 60:
      return FP_BLEND_NODE;
      
    default:
      report_fatal_error ("Instruction type not associated with a node");
  }
  
  return InstructionType;
}





//===----------------------------------------------------------------------===//
//        Routines to analyze DAG cycle by cycle and calculate spans
//===----------------------------------------------------------------------===//



bool
DynamicAnalysis::IsEmptyLevel (unsigned ExecutionResource, uint64_t Level, bool & IsInAvailableCyclesTree,
                               bool & IsInFullOccupancyCyclesTree, bool WithPrefetch)
{
  
  // bool IsInAvailableCyclesTree = false;
  // bool IsInFullOccupancyCyclesTree = false;
  
  if (ExecutionResource <= nExecutionUnits) {
    if (AvailableCyclesTree[ExecutionResource] != NULL) {
      AvailableCyclesTree[ExecutionResource] = splay (Level, AvailableCyclesTree[ExecutionResource]);
      if (Level == AvailableCyclesTree[ExecutionResource]->key) {
        if (AvailableCyclesTree[ExecutionResource]->issueOccupancy != 0
            || (WithPrefetch && AvailableCyclesTree[ExecutionResource]->occupancyPrefetch != 0)) {
          IsInAvailableCyclesTree = true;
          return false;
        }
      }
    }
  }
  else
    IsInAvailableCyclesTree = false;
  

  
#ifdef EFF_TBV
  IsInFullOccupancyCyclesTree = FullOccupancyCyclesTree[ExecutionResource].get_node (Level);
#else
  int TreeChunk = Level / SplitTreeRange;
  IsInFullOccupancyCyclesTree = FullOccupancyCyclesTree[TreeChunk].get_node (Level, ExecutionResource);
#endif
  if (IsInFullOccupancyCyclesTree == false && IsInAvailableCyclesTree == false) {
    return true;
  }
  else
    return false;
}


/* An alternative to optimize calculateSpan could be merging the
 AvailableCyclesTree and FullOccupancyCyclesTree and doing and
 ca inorder/ postoder travesal */
uint64_t DynamicAnalysis::CalculateSpan (int ResourceType)
{
  
  uint64_t Span = 0;
  bool   IsInAvailableCyclesTree = false;
  bool   IsInFullOccupancyCyclesTree = false;
  
  
  if (ScalarInstructionsCountExtended[ResourceType] != 0 && VectorInstructionsCountExtended[ResourceType] != 0) {
    if (ExecutionUnitsThroughput[ResourceType] != INF
        && ExecutionUnitsParallelIssue[ResourceType] != INF
        && ExecutionUnitsLatency[ResourceType] <
        AccessWidths[ResourceType] * VectorWidth / (ExecutionUnitsThroughput[ResourceType] *
                                                    ExecutionUnitsParallelIssue[ResourceType]))
      report_fatal_error
      ("Error calculating span because there are mixed scalar and vector instructions and issue cycle granularity is larger than latency\n");
  }
  //If there are instructions of this type....
  if (InstructionsCountExtended[ResourceType] > 0) {
    
    uint64_t Latency = ExecutionUnitsLatency[ResourceType];
    uint64_t First = FirstNonEmptyLevel[ResourceType];
    uint64_t DominantLevel = First;
    uint64_t LastCycle = LastIssueCycleVector[ResourceType];
    
#ifdef DEBUG_SPAN_CALCULATION
    DEBUG (dbgs () << "First  " << First << "\n");
    DEBUG (dbgs () << "Latency  " << Latency << "\n");
    DEBUG (dbgs () << "LastCycle  " << LastCycle << "\n");
    DEBUG (dbgs () << "Increasing span to  " << Latency << "\n");
#endif
    
    Span += Latency;
    
    //Start from next level to first non-emtpy level
    //  for(unsigned i=First+IssueCycleGranularity; i <= LastCycle; i+=IssueCycleGranularity){
    for (unsigned i = First + 1; i <= LastCycle; i += 1) {
      //Check whether there is instruction scheduled in this cycle!
      IsInAvailableCyclesTree = false;
      IsInFullOccupancyCyclesTree = false;
      if (IsEmptyLevel (ResourceType, i, IsInAvailableCyclesTree, IsInFullOccupancyCyclesTree) == false) {
        if (DominantLevel + Latency != 0 && i <= DominantLevel + Latency - 1) {
          if (i + Latency > DominantLevel + Latency && Latency != 0) {
#ifdef DEBUG_SPAN_CALCULATION
            DEBUG (dbgs () << "Increasing Span by the difference " <<
                   ((i + Latency) - max ((DominantLevel + Latency), (uint64_t) 1)) << "\n");
#endif
            Span += ((i + Latency) - max ((DominantLevel + Latency), (uint64_t) 1));
            DominantLevel = i;
          }
        }
        else {
#ifdef DEBUG_SPAN_CALCULATION
          DEBUG (dbgs () << "Increasing Span by " << Latency << "\n");
#endif
          Span += Latency;
          DominantLevel = i;
        }
      }
    }
  }
  return Span;
}


uint64_t
DynamicAnalysis::GetLastIssueCycle (unsigned ExecutionResource, bool WithPrefetch)
{
  
  Tree < uint64_t > *NodeAvailable = NULL;
  bool isPrefetchType = false;
  unsigned IssueCycleGranularity = IssueCycleGranularities[ExecutionResource];
  uint64_t LastCycle = InstructionsLastIssueCycle[ExecutionResource];
  
#ifdef DEBUG_GENERIC
  DEBUG (dbgs () << "Last cycle in InstructionLastIssueCycle for resource " << GetResourceName (ExecutionResource) << ": " <<
         LastCycle << "\n");
#endif
  if (ExecutionResource <= nExecutionUnits) {
    
    AvailableCyclesTree[ExecutionResource] = splay (LastCycle, AvailableCyclesTree[ExecutionResource]);
    NodeAvailable = AvailableCyclesTree[ExecutionResource];
    
    

    if (isPrefetchType) {
#ifdef EFF_TBV
      if ((NodeAvailable != NULL && NodeAvailable->key == LastCycle && NodeAvailable->occupancyPrefetch == 0)
          || (FullOccupancyCyclesTree[ExecutionResource].get_node_nb (LastCycle))) {
        
#else
        int TreeChunk = LastCycle / SplitTreeRange;
        if ((NodeAvailable != NULL && NodeAvailable->key == LastCycle && NodeAvailable->occupancyPrefetch == 0)
            || (FullOccupancyCyclesTree[TreeChunk].get_node_nb (LastCycle, ExecutionResource))) {
          
#endif
          
#ifdef DEBUG_GENERIC
          DEBUG (dbgs () << "Reducing last cycle " << LastCycle << " by issuegranularity\n");
#endif
          LastCycle = LastCycle - /*1 */ IssueCycleGranularity;
#ifdef DEBUG_GENERIC
          DEBUG (dbgs () << "LastCycle " << LastCycle << "\n");
#endif
#ifdef EFF_TBV
        }
#else
      }
#endif
    }
    else {
      if (NodeAvailable != NULL && NodeAvailable->key == LastCycle && NodeAvailable->issueOccupancy == 0) {
#ifdef DEBUG_GENERIC
        DEBUG (dbgs () << "Reducing last cycle " << LastCycle << " by issuegranularity\n");
#endif
        LastCycle = LastCycle - /*1 */ IssueCycleGranularity;
#ifdef DEBUG_GENERIC
        DEBUG (dbgs () << "LastCycle " << LastCycle << "\n");
#endif
      }
    }
  }
  return LastCycle;
}



unsigned
DynamicAnalysis::CalculateGroupSpan (vector < int >&ResourcesVector, bool WithPrefetch, bool ForceUnitLatency)
{
  
  unsigned Span = 0;
  unsigned MaxLatency = 0;
  uint64_t First = 0;
  bool EmptyLevel = true;
  bool IsGap = false;
  int NResources = ResourcesVector.size ();
  uint64_t LastCycle = 0;
  uint64_t ResourceLastCycle = 0;
  unsigned MaxLatencyLevel = 0;
  unsigned ResourceType = 0;
  unsigned AccessWidth = 0;
  bool IsInAvailableCyclesTree = false;
  bool IsInFullOccupancyCyclesTree = false;
  unsigned SpanIncrease = 0;
  

#ifdef DEBUG_SPAN_CALCULATION
  DEBUG (dbgs () << "Resources that contribute to Span:\n");
  for (int j = 0; j < NResources; j++) {
    DEBUG (dbgs () << ResourcesVector[j] << "\n");
  }
#endif
  
  //Determine first non-empty level and LastCycle
  for (int j = 0; j < NResources; j++) {
    
    ResourceType = ResourcesVector[j];
    
    if (InstructionsCountExtended[ResourceType] > 0) {
#ifdef DEBUG_SPAN_CALCULATION
      DEBUG (dbgs () << "There are instructions of type " << ResourceType << "\n");
#endif
      AccessWidth = AccessWidths[ResourceType];
      
      if (EmptyLevel == true) {	// This will be only executed the first time of a non-empty level
        EmptyLevel = false;
        First = FirstNonEmptyLevel[ResourceType];
        
        if (ExecutionUnitsThroughput[ResourceType] == INF) {
          MaxLatency = ExecutionUnitsLatency[ResourceType];
        }
        else
          MaxLatency =
          max (ExecutionUnitsLatency[ResourceType],
               (unsigned) ceil (AccessWidth / ExecutionUnitsThroughput[ResourceType]));
      }
      else {
        if (First == FirstNonEmptyLevel[ResourceType]) {
          if (ExecutionUnitsThroughput[ResourceType] == INF) {
            MaxLatency = max (MaxLatency, ExecutionUnitsLatency[ResourceType]);
          }
          else
            MaxLatency =
            max (MaxLatency,
                 max (ExecutionUnitsLatency[ResourceType],
                      (unsigned) ceil (AccessWidth / ExecutionUnitsThroughput[ResourceType])));
        }
        else {
          First = min (First, FirstNonEmptyLevel[ResourceType]);
          {
            if (First == FirstNonEmptyLevel[ResourceType]) {
              if (ExecutionUnitsThroughput[ResourceType] == INF) {
                MaxLatency = ExecutionUnitsLatency[ResourceType];
              }
              else
                MaxLatency =
                max (ExecutionUnitsLatency[ResourceType],
                     (unsigned) ceil (AccessWidth / ExecutionUnitsThroughput[ResourceType]));
            }
          }
        }
      }
      if (ForceUnitLatency == true)
        MaxLatency = 1;
      
      ResourceLastCycle = LastIssueCycleVector[ResourceType];
      
#ifdef DEBUG_SPAN_CALCULATION
      DEBUG (dbgs () << "Calling GetLastIssueCycle with args " << ResourceType << "  " << WithPrefetch << "\n");
      DEBUG (dbgs () << "Last cycle returned from ResourceLastCycle " << ResourceLastCycle << "\n");
      DEBUG (dbgs () << "First non-empty level  " << First << "\n");
      
#endif
      LastCycle = max (LastCycle, ResourceLastCycle);
      
#ifdef SOURCE_CODE_ANALYSIS
      if (NResources == 1 && ResourceType < nExecutionUnits) {
        //unsigned TreeChunk = GetTreeChunk(First);
        // TODO: Commented these two lines. Why no we need them?
        // FullOccupancyCyclesTree[TreeChunk] = splay(First,  FullOccupancyCyclesTree[TreeChunk]);
        AvailableCyclesTree[ResourceType] = splay (First, AvailableCyclesTree[ResourceType]);
        CollectSourceCodeLineStatistics (ResourceType, First, MaxLatency, MaxLatency - 1, true, true);
      }
      
      
      if (NResources == 1 && ResourceType >= RS_STALL) {
        // unsigned TreeChunk = GetTreeChunk(First);
        // FullOccupancyCyclesTree[TreeChunk] = splay(First,  FullOccupancyCyclesTree[TreeChunk]);
        CollectSourceCodeLineStatistics (ResourceType, First, MaxLatency, MaxLatency - 1, true, false);
      }
      
      
#endif
      
    }
  }
  
#ifdef DEBUG_SPAN_CALCULATION
  DEBUG (dbgs () << "First non-empty level  " << First << "\n");
  DEBUG (dbgs () << "MaxLatency  " << MaxLatency << "\n");
  DEBUG (dbgs () << "LastCycle  " << LastCycle << "\n");
#endif
  unsigned DominantLevel = First;
  
  
  if (EmptyLevel == false) {
    Span += MaxLatency;
    
    
    for (uint64_t i = First + 1; i <= LastCycle; i++) {
      // For sure there is at least resource for which this level is not empty.
#ifdef DEBUG_SPAN_CALCULATION
      DEBUG (dbgs () << "i =   " << i << "\n");
#endif
      //Determine MaxLatency of Level
      MaxLatencyLevel = 0;
      for (int j = 0; j < NResources; j++) {
        ResourceType = ResourcesVector[j];
        
        if (i <= LastIssueCycleVector[ResourceType]
            ) {
          IsInAvailableCyclesTree = false;
          IsInFullOccupancyCyclesTree = false;
          if (IsEmptyLevel (ResourceType, i, IsInAvailableCyclesTree, IsInFullOccupancyCyclesTree, WithPrefetch) == false) {
            IsGap = false;
            // MaxLatencyLevel = max(MaxLatencyLevel, GetInstructionLatency(ResourcesVector[j]));
            if (ForceUnitLatency == true) {
              MaxLatencyLevel = 1;
            }
            else {
              AccessWidth = AccessWidths[ResourceType];
              
              if (ExecutionUnitsThroughput[ResourceType] == INF) {
                MaxLatencyLevel = max (MaxLatencyLevel, ExecutionUnitsLatency[ResourceType]);
                
                
              }
              else {
                MaxLatencyLevel =
                max (MaxLatencyLevel,
                     max (ExecutionUnitsLatency[ResourceType],
                          (unsigned) ceil (AccessWidth / ExecutionUnitsThroughput[ResourceType])));
              }
            }
          }
        }
      }
      
#ifdef DEBUG_SPAN_CALCULATION
      DEBUG (dbgs () << "MaxLatencyLevel  " << MaxLatencyLevel << "\n");
#endif
      
      //That is, only if there are instructions scheduled in this cycle
      if (MaxLatencyLevel != 0) {
        
        // Add the first condition because if Latency=0 is allowed, it can happen
        // that DominantLevel+MaxLatency-1 is a negative number, so the loop
        // is entered incorrectly.
        if (DominantLevel + MaxLatency != 0 && i <= DominantLevel + MaxLatency - 1) {
          
          if (i + MaxLatencyLevel > DominantLevel + MaxLatency && MaxLatencyLevel != 0) {
            
            SpanIncrease = ((i + MaxLatencyLevel) - max ((DominantLevel + MaxLatency), (unsigned) 1));
            
            
            
#ifdef DEBUG_SPAN_CALCULATION
            DEBUG (dbgs () << "Increasing Span by the difference " << SpanIncrease << "\n");
#endif
            
            Span += SpanIncrease;
            DominantLevel = i;
            MaxLatency = MaxLatencyLevel;
          }
        }
        else {
          
#ifdef DEBUG_SPAN_CALCULATION
          DEBUG (dbgs () << "Increasing Span by " << MaxLatencyLevel << "\n");
#endif
          SpanIncrease = MaxLatencyLevel - 1;
          Span += MaxLatencyLevel;
          DominantLevel = i;
          MaxLatency = MaxLatencyLevel;
        }
        
#ifdef SOURCE_CODE_ANALYSIS
        if (NResources == 1 && (ResourceType < nExecutionUnits || ResourceType >= RS_STALL)) {
          AvailableCyclesTree[ResourceType] = splay (i, AvailableCyclesTree[ResourceType]);
          CollectSourceCodeLineStatistics (ResourceType, i, MaxLatencyLevel, SpanIncrease, IsInFullOccupancyCyclesTree,
                                           IsInAvailableCyclesTree);
        }
        
#endif
      }
      else {
        if (i > DominantLevel + MaxLatency - 1) {
          if (NResources == 1 && IsGap == false) {
            SpanGaps[ResourceType]++;
#ifdef DEBUG_SPAN_CALCULATION
            DEBUG (dbgs () << "Increasing span gaps for resource " << GetResourceName (ResourceType) << "\n");
#endif
            IsGap = true;
          }
        }
      }
      
    }
  }
  
  return Span;
}



unsigned
DynamicAnalysis::CalculateIssueSpan (vector < int >&ResourcesVector)
{
  
  unsigned Span = 0;
  unsigned MaxLatency = 0;
  uint64_t First = 0;
  bool EmptyLevel = true;
  int NResources = ResourcesVector.size ();
  uint64_t LastCycle = 0;
  uint64_t ResourceLastCycle = 0;
  unsigned MaxLatencyLevel = 0;
  unsigned ResourceType = 0;
  unsigned AccessWidth = 0;
  unsigned TmpLatency = 0;
  bool IsInAvailableCyclesTree = false;
  bool IsInFullOccupancyCyclesTree = false;
  
#ifdef DEBUG_SPAN_CALCULATION
  DEBUG (dbgs () << "Resources that contribute to Span:\n");
  for (int j = 0; j < NResources; j++) {
    DEBUG (dbgs () << ResourcesVector[j] << "\n");
  }
#endif
  
  //Determine first non-empty level and LastCycle
  for (int j = 0; j < NResources; j++) {
    
    ResourceType = ResourcesVector[j];
    
    if (InstructionsCountExtended[ResourceType] > 0) {
      
      AccessWidth = AccessWidths[ResourceType];
      if (ExecutionUnitsThroughput[ResourceType] == INF)
        TmpLatency = 1;
      else
        TmpLatency = ceil (AccessWidth / ExecutionUnitsThroughput[ResourceType]);
#ifdef DEBUG_SPAN_CALCULATION
      DEBUG (dbgs () << "AccessWidth " << AccessWidth << "\n");
      DEBUG (dbgs () << "There are instructions of type " << ResourceType << "\n");
      DEBUG (dbgs () << "How many? " << InstructionsCountExtended[ResourceType] << "\n");
#endif
      if (EmptyLevel == true) {	// This will be only executed the first time of a non-empty level
        EmptyLevel = false;
        First = FirstNonEmptyLevel[ResourceType];
        MaxLatency = TmpLatency;
      }
      else {
        if (First == FirstNonEmptyLevel[ResourceType])
          // MaxLatency = max(MaxLatency,(unsigned)ceil(AccessWidth/ExecutionUnitsThroughput[ResourceType]));
          MaxLatency = max (MaxLatency, TmpLatency);
        else {
          First = min (First, FirstNonEmptyLevel[ResourceType]);
          {
            if (First == FirstNonEmptyLevel[ResourceType])
              //MaxLatency = ceil(AccessWidth/ExecutionUnitsThroughput[ResourceType]);
              MaxLatency = TmpLatency;
          }
        }
      }
#ifdef DEBUG_SPAN_CALCULATION
      DEBUG (dbgs () << "ResourceType " << ResourceType << "\n");
      DEBUG (dbgs () << "LastIssueCycleVector size " << LastIssueCycleVector.size () << "\n");
      DEBUG (dbgs () << "LastIssueCycleVector[ResourceType] " << LastIssueCycleVector[ResourceType] << "\n");
#endif
      ResourceLastCycle = LastIssueCycleVector[ResourceType];
      
#ifdef DEBUG_SPAN_CALCULATION
      DEBUG (dbgs () << "Last cycle returned from ResourceLastCycle " << ResourceLastCycle << "\n");
#endif
      LastCycle = max (LastCycle, ResourceLastCycle);
    }
  }
  
  
#ifdef DEBUG_SPAN_CALCULATION
  DEBUG (dbgs () << "First non-empty level  " << First << "\n");
  DEBUG (dbgs () << "MaxLatency  " << MaxLatency << "\n");
  DEBUG (dbgs () << "LastCycle  " << LastCycle << "\n");
#endif
  unsigned DominantLevel = First;
  if (EmptyLevel == false) {
    Span += MaxLatency;
    
    //Start from next level to first non-emtpy level
    for (unsigned i = First + 1; i <= LastCycle; i++) {
      //Determine MaxLatency of Level
      MaxLatencyLevel = 0;
      for (int j = 0; j < NResources; j++) {
        ResourceType = ResourcesVector[j];
        
        if (i <= LastIssueCycleVector[ResourceType]
            ) {
          IsInAvailableCyclesTree = false;
          IsInFullOccupancyCyclesTree = false;
          if (IsEmptyLevel (ResourceType, i, IsInAvailableCyclesTree, IsInFullOccupancyCyclesTree, false) == false) {
            AccessWidth = AccessWidths[ResourceType];
            if (ExecutionUnitsThroughput[ResourceType] == INF)
              TmpLatency = 1;
            else
              TmpLatency = ceil (AccessWidth / ExecutionUnitsThroughput[ResourceType]);
            MaxLatencyLevel = max (MaxLatencyLevel, TmpLatency);
          }
        }
      }
      
#ifdef DEBUG_SPAN_CALCULATION
      DEBUG (dbgs () << "i =   " << i << "\n");
      DEBUG (dbgs () << "MaxLatencyLevel  " << MaxLatencyLevel << "\n");
#endif
      
      //That is, only if there are instructions scheduled in this cycle
      if (MaxLatencyLevel != 0) {
        if (i <= DominantLevel + MaxLatency - 1) {
          
          
          if (i + MaxLatencyLevel > DominantLevel + MaxLatency && MaxLatencyLevel != 0) {
#ifdef DEBUG_SPAN_CALCULATION
            DEBUG (dbgs () << "Increasing Span by the difference " <<
                   ((i + MaxLatencyLevel) - max ((DominantLevel + MaxLatency), (unsigned) 1)) << "\n");
#endif
            
            Span += ((i + MaxLatencyLevel) - max ((DominantLevel + MaxLatency), (unsigned) 1));
            DominantLevel = i;
            MaxLatency = MaxLatencyLevel;
          }
        }
        else {
          
#ifdef DEBUG_SPAN_CALCULATION
          DEBUG (dbgs () << "Increasing Span by " << MaxLatencyLevel << "\n");
#endif
          
          Span += MaxLatencyLevel;
          DominantLevel = i;
          MaxLatency = MaxLatencyLevel;
        }
      }
    }
  }

#ifdef DEBUG_SPAN_CALCULATION
  DEBUG (dbgs () << "Span = " << Span << "\n");
#endif
  return Span;
}

//===----------------------------------------------------------------------===//
//                      Routine for source code analysis
//===----------------------------------------------------------------------===//



#ifdef SOURCE_CODE_ANALYSIS
void
DynamicAnalysis::CollectSourceCodeLineStatistics (uint64_t ResourceType,
                                                  uint64_t Cycle, uint64_t MaxLatencyLevel, uint64_t SpanIncrease,
                                                  bool IsInFullOccupancyCyclesTree, bool IsInAvailableCyclesTree)
{
  
  uint64_t Line = 0;
  unsigned Resource = 0;
  typedef unordered_map < uint64_t, set < uint64_t > >::iterator it_type;
  
#ifdef EFF_TBV
  GetTreeChunk (Cycle, ResourceType);
#else
  unsigned TreeChunk = GetTreeChunk (Cycle);
#endif
  
  
  
  if (FullOccupancyCyclesTree[TreeChunk].get_node (Cycle, ResourceType)) {
    
    vector < pair < unsigned, unsigned >>SourceCodeLinesOperationPair =
    FullOccupancyCyclesTree[TreeChunk].get_source_code_lines (Cycle);
    
    for (std::vector < pair < unsigned, unsigned >>::iterator it = SourceCodeLinesOperationPair.begin ();
         it != SourceCodeLinesOperationPair.end (); ++it) {
      
      Line = (*it).first;
      Resource = (*it).second;
      
      // SourceCodeLineInfo[Line] contains a set of the cycles associated to this line.
      // We want all the cycles associated to a line for the fraction of the total
      // span this line contributes to
      
      
      if (ResourceType == Resource) {
#ifdef DEBUG_SOURCE_CODE_LINE_ANALYSIS
        DEBUG (dbgs () << "ResourceType " << GetResourceName (ResourceType) << "\n");
        DEBUG (dbgs () << "Operation of this line " << GetResourceName (Resource) << "\n");
        DEBUG (dbgs () << "Line " << Line << " contributes issuing or stall to this cycle (" << Cycle << ")\n");
#endif
        
        SourceCodeLineInfo[Line].insert (Cycle);
        // SourceCodeLineInfoBreakdown[Line] is a vector.
        if (SourceCodeLineInfoBreakdown[Line].size () == 0) {
          for (int z = 0; z <= 24; z++) {
            SourceCodeLineInfoBreakdown[Line].push_back (0);
          }
        }
#ifdef DEBUG_SOURCE_CODE_LINE_ANALYSIS
        DEBUG (dbgs () << "Increasing  SourceCodeLineInfoBreakdown of Line " << Line << " at position " <<
               GetPositionSourceCodeLineInfoVector (ResourceType) << "\n");
#endif
        SourceCodeLineInfoBreakdown[Line]
        [GetPositionSourceCodeLineInfoVector (ResourceType)]++;
        // For latency cycles.
        for (unsigned k = Cycle + 1; k < Cycle + MaxLatencyLevel; k++) {
          SourceCodeLineInfo[Line].insert (k);
        }
        
        
        for (unsigned k = 0; k < SpanIncrease; k++) {
#ifdef DEBUG_SOURCE_CODE_LINE_ANALYSIS
          DEBUG (dbgs () << "Increasing  SourceCodeLineInfoBreakdown at position " <<
                 GetPositionSourceCodeLineInfoVector (ResourceType)
                 + 1 << "\n");
#endif
          SourceCodeLineInfoBreakdown[Line]
          [GetPositionSourceCodeLineInfoVector (ResourceType) + 1]++;
        }
        
        if (SpanIncrease != MaxLatencyLevel && SourceCodeLineInfoBreakdown[Line]
            [GetPositionSourceCodeLineInfoVector (ResourceType) + 1] != 0) {
          SourceCodeLineInfoBreakdown[Line]
          [GetPositionSourceCodeLineInfoVector (ResourceType) + 1]--;
#ifdef DEBUG_SOURCE_CODE_LINE_ANALYSIS
          DEBUG (dbgs () << "Decreasing one cycle latency\n");
#endif
        }
        
        
      }
      
    }
  }
  
  if (IsInAvailableCyclesTree == true && AvailableCyclesTree[ResourceType] != NULL
      && AvailableCyclesTree[ResourceType]->key == Cycle) {
    
    // For every line in the source code
    
    for (auto it = AvailableCyclesTree[ResourceType]->SourceCodeLinesOperationPair.begin ();
         it != AvailableCyclesTree[ResourceType]->SourceCodeLinesOperationPair.end (); ++it) {
      
      
      Line = (*it).first;
      
#ifdef DEBUG_SOURCE_CODE_LINE_ANALYSIS
      DEBUG (dbgs () << "Line " << Line << " contributes issuing or stall to this cycle (" << Cycle << ")\n");
#endif
      // SourceCodeLineInfo[Line] contains a set of the cycles associated to this line.
      // We want all the cycles associated to a line for the fraction of the total
      // span this line contributes to
      
      SourceCodeLineInfo[Line].insert (Cycle);
      
      // SourceCodeLineInfoBreakdown[Line] is a vector.
      if (SourceCodeLineInfoBreakdown[Line].size () == 0) {
        for (int z = 0; z <= 21; z++) {
          SourceCodeLineInfoBreakdown[Line].push_back (0);
        }
      }
#ifdef DEBUG_SOURCE_CODE_LINE_ANALYSIS
      DEBUG (dbgs () << "Increasing  SourceCodeLineInfoBreakdown at position " <<
             GetPositionSourceCodeLineInfoVector (ResourceType) << "\n");
#endif
      SourceCodeLineInfoBreakdown[Line]
      [GetPositionSourceCodeLineInfoVector (ResourceType)]++;
      
      if (SpanIncrease != MaxLatencyLevel && SourceCodeLineInfoBreakdown[Line]
          [GetPositionSourceCodeLineInfoVector (ResourceType) + 1] != 0) {
        SourceCodeLineInfoBreakdown[Line]
        [GetPositionSourceCodeLineInfoVector (ResourceType) + 1]--;
#ifdef DEBUG_SOURCE_CODE_LINE_ANALYSIS
        DEBUG (dbgs () << "Decreasing one cycle latency\n");
#endif
      }
      
      // For latency cycles.
      for (unsigned k = Cycle + 1; k < Cycle + MaxLatencyLevel; k++) {
        SourceCodeLineInfo[Line].insert (k);
      }
      
      for (unsigned k = 0; k < SpanIncrease; k++) {
#ifdef DEBUG_SOURCE_CODE_LINE_ANALYSIS
        DEBUG (dbgs () << "Increasing  SourceCodeLineInfoBreakdown at position " <<
               GetPositionSourceCodeLineInfoVector (ResourceType) + 1 << "\n");
#endif
        SourceCodeLineInfoBreakdown[Line]
        [GetPositionSourceCodeLineInfoVector (ResourceType) + 1]++;
      }
      
    }
  }
  
}


#endif


//===----------------------------------------------------------------------===//
//                      OoO Buffers routines
//===----------------------------------------------------------------------===//


uint64_t
DynamicAnalysis::GetMinIssueCycleReservationStation ()
{
  
  vector < uint64_t >::iterator it;
  
  uint64_t
  MinIssueCycle = ReservationStationIssueCycles.front ();
  for (it = ReservationStationIssueCycles.begin (); it != ReservationStationIssueCycles.end (); ++it)
    MinIssueCycle = min (MinIssueCycle, *it);
  
  return MinIssueCycle;
}



uint64_t
DynamicAnalysis::GetMinCompletionCycleLoadBuffer ()
{
  
  vector < uint64_t >::iterator it;
  
  uint64_t
  MinCompletionCycle = LoadBufferCompletionCycles.front ();
  for (it = LoadBufferCompletionCycles.begin (); it != LoadBufferCompletionCycles.end (); ++it) {
    MinCompletionCycle = min (MinCompletionCycle, *it);
  }
  return MinCompletionCycle;
}


uint64_t
DynamicAnalysis::GetMinCompletionCycleLoadBufferTree ()
{
  
  return MinLoadBuffer;
}




uint64_t DynamicAnalysis::GetMinCompletionCycleStoreBuffer ()
{
  
  vector < uint64_t >::iterator it;
  
  uint64_t
  MinCompletionCycle = StoreBufferCompletionCycles.front ();
  for (it = StoreBufferCompletionCycles.begin (); it != StoreBufferCompletionCycles.end (); ++it) {
    MinCompletionCycle = min (MinCompletionCycle, *it);
  }
  return MinCompletionCycle;
}



uint64_t DynamicAnalysis::GetMinCompletionCycleLineFillBuffer ()
{
  
  vector < uint64_t >::iterator it;
  
  uint64_t
  MinCompletionCycle = LineFillBufferCompletionCycles.front ();
  for (it = LineFillBufferCompletionCycles.begin (); it != LineFillBufferCompletionCycles.end (); ++it) {
    MinCompletionCycle = min (MinCompletionCycle, *it);
  }
  return MinCompletionCycle;
}



bool DynamicAnalysis::isStallCycle (int ResourceType, uint64_t Level)
{
  
  StallCycles[ResourceType] = splay (Level, StallCycles[ResourceType]);
  if (StallCycles[ResourceType] == NULL) {
    return false;
  }
  else {
    return true;
  }
}


void
DynamicAnalysis::RemoveFromReservationStation (uint64_t Cycle)
{
  
  LessThanOrEqualValuePred Predicate = { Cycle };
  ReservationStationIssueCycles.
  erase (std::remove_if (ReservationStationIssueCycles.begin (), ReservationStationIssueCycles.end (), Predicate),
         ReservationStationIssueCycles.end ());
}



void
DynamicAnalysis::RemoveFromReorderBuffer (uint64_t Cycle)
{
  
  while (!ReorderBufferCompletionCycles.empty ()
         && ReorderBufferCompletionCycles.front () <= Cycle)
    ReorderBufferCompletionCycles.pop_front ();
}



void
DynamicAnalysis::RemoveFromLoadBuffer (uint64_t Cycle)
{
  
  LessThanOrEqualValuePred Predicate = { Cycle };
  LoadBufferCompletionCycles.
  erase (std::remove_if (LoadBufferCompletionCycles.begin (), LoadBufferCompletionCycles.end (), Predicate),
         LoadBufferCompletionCycles.end ());
}

void
DynamicAnalysis::RemoveFromLoadBufferTree (uint64_t Cycle)
{
  
  
#ifdef DEBUG_OOO_BUFFERS
  DEBUG(dbgs() << "Removing element with Cycle "<< Cycle <<" from LoadBufferTree\n");
  DEBUG(dbgs() << "Size before removing "<< node_size(LoadBufferCompletionCyclesTree) <<"\n");
#endif
  bool CycleFound = true;
  // TODO: This may be an infinite loop??
  while (LoadBufferCompletionCyclesTree!=NULL && (CycleFound == true || Cycle >= MinLoadBuffer)) {
    
    
    //  if (LoadBufferCompletionCyclesTree != NULL) {
    LoadBufferCompletionCyclesTree = splay (Cycle, LoadBufferCompletionCyclesTree);
    
    if (LoadBufferCompletionCyclesTree->key == Cycle) {	// If Cycle found
      LoadBufferCompletionCyclesTree->left = NULL;
#ifdef DEBUG_OOO_BUFFERS
      DEBUG(dbgs() << "Cycle found, so actually removing the element\n");
#endif
      LoadBufferCompletionCyclesTree = delete_node (Cycle, LoadBufferCompletionCyclesTree);
      // If we remove the minimum, the resulting tree has as node the
      // successor of the minimum, which is the next minimum -> This is not
      // true after we have splayed and the minimum is in the root.
      // The condition cannot be if (Cycle == MinLoadBuffer && LoadBufferCompletionCyclesTree != NULL)
      // Because we may have skipped some cycles, and we are not actually removing the minimum
      if (Cycle >= MinLoadBuffer && LoadBufferCompletionCyclesTree != NULL) {
        
        MinLoadBuffer = min (LoadBufferCompletionCyclesTree);
#ifdef DEBUG_OOO_BUFFERS
        
        DEBUG(dbgs() << "Removing element from Load Buffer Tree. Updating MinLoadBuffer to "<<MinLoadBuffer<<"\n");
#endif
        // MinLoadBuffer = LoadBufferCompletionCyclesTree->key;
      }
      if(LoadBufferCompletionCyclesTree==NULL)
        break;
    }
    else {
      //  CycleFound = false;
      if(Cycle >= MinLoadBuffer)
        Cycle--;
      else
        CycleFound = false;
    }
    
    
    CycleFound = false;
    
  }
  
#ifdef DEBUG_OOO_BUFFERS
  DEBUG(dbgs() << "Size after removing "<< node_size(LoadBufferCompletionCyclesTree) <<"\n");
#endif
  
}



void
DynamicAnalysis::RemoveFromStoreBuffer (uint64_t Cycle)
{
  
  LessThanOrEqualValuePred Predicate = { Cycle };
  StoreBufferCompletionCycles.
  erase (std::remove_if (StoreBufferCompletionCycles.begin (), StoreBufferCompletionCycles.end (), Predicate),
         StoreBufferCompletionCycles.end ());
}



void
DynamicAnalysis::RemoveFromLineFillBuffer (uint64_t Cycle)
{
  
  LessThanOrEqualValuePred Predicate = { Cycle };
  LineFillBufferCompletionCycles.erase (std::remove_if (LineFillBufferCompletionCycles.begin (),
                                                        LineFillBufferCompletionCycles.end (), Predicate),
                                        LineFillBufferCompletionCycles.end ());
}



void
DynamicAnalysis::RemoveFromDispatchToLoadBufferQueue (uint64_t Cycle)
{
  
  StructMemberLessThanOrEqualThanValuePred Predicate = { Cycle };
  DispatchToLoadBufferQueue.
  erase (std::remove_if (DispatchToLoadBufferQueue.begin (), DispatchToLoadBufferQueue.end (), Predicate),
         DispatchToLoadBufferQueue.end ());
}



void
DynamicAnalysis::RemoveFromDispatchToStoreBufferQueue (uint64_t Cycle)
{
  StructMemberLessThanOrEqualThanValuePred Predicate = { Cycle };
  DispatchToStoreBufferQueue.
  erase (std::remove_if (DispatchToStoreBufferQueue.begin (), DispatchToStoreBufferQueue.end (), Predicate),
         DispatchToStoreBufferQueue.end ());
}



void
DynamicAnalysis::RemoveFromDispatchToLineFillBufferQueue (uint64_t Cycle)
{
  
  StructMemberLessThanOrEqualThanValuePred Predicate = { Cycle };
  DispatchToLineFillBufferQueue.
  erase (std::remove_if (DispatchToLineFillBufferQueue.begin (), DispatchToLineFillBufferQueue.end (), Predicate),
         DispatchToLineFillBufferQueue.end ());
}




void
DynamicAnalysis::DispatchToLoadBuffer (uint64_t Cycle)
{
  vector < InstructionDispatchInfo >::iterator it = DispatchToLoadBufferQueue.begin ();
  for (; it != DispatchToLoadBufferQueue.end ();) {
    if ((*it).IssueCycle == InstructionFetchCycle) {
      //Erase returns the next valid iterator => insert in LoadBuffer before it is removed
      LoadBufferCompletionCycles.push_back ((*it).CompletionCycle);
      it = DispatchToLoadBufferQueue.erase (it);
    }
    else
      ++it;
  }
}



void
DynamicAnalysis::inOrder (uint64_t i, ComplexTree < uint64_t > *n)
{
  
  bool condition = false;
  bool StopChecking = false;
  if (n == NULL){
    return;
  }else
    
    inOrder (i, n->left);
  for(std::vector<uint64_t>::iterator it = n->IssueCycles.begin(); it != n->IssueCycles.end(); ++it) {
    if (*it <= i && *it != 0) {
      condition = true;
#ifdef DEBUG_OOO_BUFFERS
      DEBUG (dbgs () << "Size of issue cycles "<<  n->IssueCycles.size()<<"\n");
#endif
      break;
    }
  }
  if(condition==true){
    

    
#ifdef DEBUG_OOO_BUFFERS
    DEBUG (dbgs () << "Cycle of inOrder "<< i << "\n");
    DEBUG (dbgs () << "Iterating thought issue cycles\n");
    DEBUG (dbgs () << "Size of issue cycles "<<  n->IssueCycles.size()<<"\n");
#endif
    for(std::vector<uint64_t>::iterator it = n->IssueCycles.begin(); (!StopChecking && it != n->IssueCycles.end()); ++it) {
#ifdef DEBUG_OOO_BUFFERS
      DEBUG (dbgs () << "Issue cycle "<< *it<<"\n");
#endif
      if (*it <= i && *it != 0) {
        // Insert only if completion cycle is not smaller than i
        if(n->key > i){
          if (node_size (LoadBufferCompletionCyclesTree) == 0) {
            
            MinLoadBuffer = n->key;
            
          }
          else {
            MinLoadBuffer = min (MinLoadBuffer, n->key);
            
          }
          
          LoadBufferCompletionCyclesTree = insert_node (n->key, LoadBufferCompletionCyclesTree);
#ifdef DEBUG_OOO_BUFFERS
          DEBUG (dbgs () << "Inserting into LoadBufferCompletionCYclesTree\n");
#endif
          
        }
        if(n->IssueCycles.size()==1){
          StopChecking = true;
        }
#ifdef DEBUG_OOO_BUFFERS
        DEBUG(dbgs() << "Storing into DispatchToLoadBufferQueueTreeCyclesToRemove pair " << n->key << ", " << i<< "\n");
#endif
        DispatchToLoadBufferQueueTreeCyclesToRemove.push_back(std::make_pair(n->key,*it));
      

      }
    }

  }
  if(n!=NULL)
    inOrder (i, n->right);
}




void
DynamicAnalysis::DispatchToLoadBufferTree (uint64_t Cycle)
{
  
  DispatchToLoadBufferQueueTreeCyclesToRemove.clear();
  
  inOrder (Cycle, DispatchToLoadBufferQueueTree);
  for(unsigned int i = 0; i< DispatchToLoadBufferQueueTreeCyclesToRemove.size();i++){
#ifdef DEBUG_OOO_BUFFERS
    DEBUG(dbgs() << "Removing  pair " << DispatchToLoadBufferQueueTreeCyclesToRemove[i].first << ", " << DispatchToLoadBufferQueueTreeCyclesToRemove[i].second<< "\n");
#endif
    
    DispatchToLoadBufferQueueTree = delete_node (DispatchToLoadBufferQueueTreeCyclesToRemove[i].first,DispatchToLoadBufferQueueTreeCyclesToRemove[i].second,
                                                 DispatchToLoadBufferQueueTree);
    
  }
  
}


void
DynamicAnalysis::DispatchToStoreBuffer (uint64_t Cycle)
{
  vector < InstructionDispatchInfo >::iterator it = DispatchToStoreBufferQueue.begin ();
  for (; it != DispatchToStoreBufferQueue.end ();) {
    if ((*it).IssueCycle == InstructionFetchCycle) {
      StoreBufferCompletionCycles.push_back ((*it).CompletionCycle);
      it = DispatchToStoreBufferQueue.erase (it);
    }
    else
      ++it;
  }
}



void
DynamicAnalysis::DispatchToLineFillBuffer (uint64_t Cycle)
{
  vector < InstructionDispatchInfo >::iterator it = DispatchToLineFillBufferQueue.begin ();
  for (; it != DispatchToLineFillBufferQueue.end ();) {
    
    if ((*it).IssueCycle == InstructionFetchCycle) {
      LineFillBufferCompletionCycles.push_back ((*it).CompletionCycle);
      it = DispatchToLineFillBufferQueue.erase (it);
    }
    else
      ++it;
  }
}



uint64_t DynamicAnalysis::FindIssueCycleWhenLineFillBufferIsFull ()
{
  
  size_t
  BufferSize = DispatchToLineFillBufferQueue.size ();
  
  if (BufferSize == 0) {
    return GetMinCompletionCycleLineFillBuffer ();
  }
  else {
    if (BufferSize >= (unsigned) LineFillBufferSize) {
      // Iterate from end-LineFillBufferSize
      uint64_t
      EarliestCompletion = DispatchToLineFillBufferQueue.back ().CompletionCycle;
      for (vector < InstructionDispatchInfo >::iterator it = DispatchToLineFillBufferQueue.end () - 1;
           it >= DispatchToLineFillBufferQueue.end () - LineFillBufferSize; --it) {
        if ((*it).CompletionCycle < EarliestCompletion) {
          EarliestCompletion = (*it).CompletionCycle;
        }
      }
      return EarliestCompletion;
    }
    else {
      sort (LineFillBufferCompletionCycles.begin (), LineFillBufferCompletionCycles.end ());
      return LineFillBufferCompletionCycles[BufferSize];
    }
  }
}



uint64_t DynamicAnalysis::FindIssueCycleWhenLoadBufferIsFull ()
{
  
  size_t
  BufferSize = DispatchToLoadBufferQueue.size ();
#ifdef DEBUG_OOO_BUFFERS
  PrintDispatchToLoadBuffer();
#endif
  if (BufferSize == 0) {
    return GetMinCompletionCycleLoadBuffer ();
  }
  else {
    // Iterate through the DispathToLoadBufferQueue and get the
    // largest dispatch cycle. The new load cannot be dispatched
    // untill all previous in Dispatch Queue have been dispatched.
    // At the same time,
    uint64_t
    EarliestDispatchCycle = 0;
    
    for (vector < InstructionDispatchInfo >::iterator it = DispatchToLoadBufferQueue.begin ();
         it != DispatchToLoadBufferQueue.end (); ++it) {
      EarliestDispatchCycle = max (EarliestDispatchCycle, (*it).IssueCycle);
    }
#ifdef DEBUG_OOO_BUFFERS
    
    DEBUG(dbgs() << "EarliestDispatchCycle " << EarliestDispatchCycle << "\n");
#endif
    //Traverse LB and count how many elements are there *smaller than or equal* EarliestDispathCycle
    unsigned
    counter = 0;
    for (vector < uint64_t >::iterator it = LoadBufferCompletionCycles.begin (); it != LoadBufferCompletionCycles.end ();
         ++it) {
      if ((*it) <= EarliestDispatchCycle)
        counter++;
    }
#ifdef DEBUG_OOO_BUFFERS
    
    DEBUG(dbgs() << "counter " << counter << "\n");
#endif
    uint64_t
    IssueCycle = 0;
    // This means that in LB, there are more loads that terminate before or in
    // my dispatch cycle -> IssueCycle is Earliest
    if (counter > BufferSize) {
      IssueCycle = EarliestDispatchCycle;
      
    }else {
      if (counter == BufferSize|| (counter < BufferSize && counter==LoadBufferCompletionCycles.size() && BufferSize > LoadBufferSize) ) {
        
        // Iterate through both, DispatchBufferQueue and LB to count how many elements are completed
        // in a cycle *larger than* EarliestDispatchCycle
        unsigned CompletedAfterCounter = 0;
        
        for (vector < InstructionDispatchInfo >::iterator it = DispatchToLoadBufferQueue.begin ();
             it != DispatchToLoadBufferQueue.end (); ++it) {
          if ((*it).CompletionCycle > EarliestDispatchCycle)
            CompletedAfterCounter++;
        }
        for (vector < uint64_t >::iterator it = LoadBufferCompletionCycles.begin (); it != LoadBufferCompletionCycles.end ();
             ++it) {
          if ((*it) > EarliestDispatchCycle)
            CompletedAfterCounter++;
        }
        
        if(CompletedAfterCounter < LoadBufferSize){
          IssueCycle = EarliestDispatchCycle;
        }else{
          // Iterate through both, DispatchBufferQueue and LB to determine the smallest
          // completion cycle which is larger than EarliestDispatchCycle.
          // Initialize with the Completion cycle of the last element of the
          // DispatchToLoadBufferQueue
          IssueCycle = DispatchToLoadBufferQueue.back ().CompletionCycle;
          for (vector < InstructionDispatchInfo >::iterator it = DispatchToLoadBufferQueue.begin ();
               it != DispatchToLoadBufferQueue.end (); ++it) {
            if ((*it).CompletionCycle > EarliestDispatchCycle)
              IssueCycle = min (IssueCycle, (*it).CompletionCycle);
          }
#ifdef DEBUG_OOO_BUFFERS
          
          DEBUG(dbgs() << "IssueCycle after iteration through DispatchToLoadBufferQueue "<< IssueCycle<<"\n");
#endif
          // We have to also iterate over the completion cycles of the LB even
          // if there are more elements in the DispatchQueue than the size
          // of the LB. Because it can happen than all the elements of the
          // DispatchQueue are complemente even before than an element in the
          // LB which is waiting very long for a resource.
          // We could, nevertheless, simplify it. We can keep the max and
          // the min completion cycle always. If the max completion cycle
          // is smaller than the EarliestDispatchCycle, then it is not necessary
          // to iterate over the LB.
          for (vector < uint64_t >::iterator it = LoadBufferCompletionCycles.begin (); it != LoadBufferCompletionCycles.end ();
               ++it) {
            //  if ((*it) > EarliestDispatchCycle + 1)
            if ((*it) > EarliestDispatchCycle)
              IssueCycle = min (IssueCycle, *it);
          }
#ifdef DEBUG_OOO_BUFFERS
          
          DEBUG(dbgs() << "IssueCycle after iteration through LoadBufferCompletionCycles"<<IssueCycle<<"\n");
#endif
        }
      }
      else {
        report_fatal_error ("Error in Dispatch to Load Buffer Queue");
        
      }
    }
    
    return IssueCycle;
    
  }
  
}


uint64_t DynamicAnalysis::FindIssueCycleWhenLoadBufferTreeIsFull ()
{
#ifdef DEBUG_OOO_BUFFERS
  
  DEBUG (dbgs () << "Finding issue cycle when load buffer tree is full\n");
#endif
  size_t
  BufferSize = node_size (DispatchToLoadBufferQueueTree);
  
  if (BufferSize == 0) {
    
    return GetMinCompletionCycleLoadBufferTree ();
  }
  else {
    uint64_t EarliestDispatchCycle = 0;
    
    uint64_t SlotsLoadBufferCompleteBeforeEarliestDispatch = 0;
    uint64_t SlotsDispatchCompleteBeforeEarliest = 0;
    uint64_t SlotsDispatchCompleteAfterEarliest = 0;
    uint64_t TotalCyclesCompleteAfter = 0;
    // Iterate through the DispathToLoadBufferQueue and get the
    // largest dispatch cycle. The new load cannot be dispatched
    // untill all previous in Dispatch Queue have been dispatched.
    //  We keep a variable, so no need to iterate
    EarliestDispatchCycle = MaxDispatchToLoadBufferQueueTree;
#ifdef DEBUG_OOO_BUFFERS
    
    DEBUG (dbgs () << "EarliestDispatchCycle " << EarliestDispatchCycle << "\n");
#endif
    
    //Traverse LB and count how many elements are there *smaller than or equal* EarliestDispathCycle
    
    
    LoadBufferCompletionCyclesTree = splay (EarliestDispatchCycle, LoadBufferCompletionCyclesTree);
#ifdef DEBUG_OOO_BUFFERS
    
    DEBUG (dbgs () << "LoadBufferCompletionCyclesTree->key " << LoadBufferCompletionCyclesTree->key << "\n");
#endif
    // When we splay, the following can happen:
    // key == Earliest: account for the size of the left node, if any, and the duplicates of key
    // key > Earliest: account for the size of the left node, if any.
    // key < Earliest: account for the size of the left node, if any, plus the duplicated.
    // If node->left is not NULL, is the size of left. If, moreover, node->key <= EarliestComplettionCycle,
    // then add also the duplicates of this node.
    if (LoadBufferCompletionCyclesTree->left != NULL)
      SlotsLoadBufferCompleteBeforeEarliestDispatch = node_size (LoadBufferCompletionCyclesTree->left);
    if(LoadBufferCompletionCyclesTree->key <= EarliestDispatchCycle)
      SlotsLoadBufferCompleteBeforeEarliestDispatch+= LoadBufferCompletionCyclesTree->duplicates;
    
    TotalCyclesCompleteAfter = LoadBufferSize - SlotsLoadBufferCompleteBeforeEarliestDispatch;
    
    // Traverse DispatchToLoadBufferQueuteTree and count how many
    // complete after my EarliestDispatchCycle, and how many earlier
    
    
    DispatchToLoadBufferQueueTree = splay (EarliestDispatchCycle, DispatchToLoadBufferQueueTree);
    if(DispatchToLoadBufferQueueTree->left!=NULL)
      SlotsDispatchCompleteBeforeEarliest = node_size(DispatchToLoadBufferQueueTree->left);
    if(DispatchToLoadBufferQueueTree->right!=NULL)
      SlotsDispatchCompleteAfterEarliest = node_size(DispatchToLoadBufferQueueTree->right);
    
    if(DispatchToLoadBufferQueueTree->key <= EarliestDispatchCycle)
      SlotsDispatchCompleteBeforeEarliest +=  DispatchToLoadBufferQueueTree->IssueCycles.size();
    else
      SlotsDispatchCompleteAfterEarliest+=  DispatchToLoadBufferQueueTree->IssueCycles.size();
#ifdef DEBUG_OOO_BUFFERS
    bool AvailableSlots = false;
    AvailableSlots = SlotsLoadBufferCompleteBeforeEarliestDispatch;
    DEBUG (dbgs () << "AvailableSlots " << AvailableSlots << "\n");
#endif
    TotalCyclesCompleteAfter+=SlotsDispatchCompleteAfterEarliest;
#ifdef DEBUG_OOO_BUFFERS
    
    
    DEBUG (dbgs () << "SlotsLoadBufferCompleteBeforeEarliestDispatch " << SlotsLoadBufferCompleteBeforeEarliestDispatch << "\n");
    DEBUG (dbgs () << "SlotsDispatchCompleteBeforeEarliest " << SlotsDispatchCompleteBeforeEarliest << "\n");
    DEBUG (dbgs () << "SlotsDispatchCompleteAfterEarliest " << SlotsDispatchCompleteAfterEarliest << "\n");
    DEBUG (dbgs () << "TotalCyclesCompleteAfter " << TotalCyclesCompleteAfter << "\n");
#endif
    uint64_t IssueCycle = 0;
    // This means that in LB, there are more loads that terminate before or in
    // my dispatch cycle -> IssueCycle is Earliest
    if(TotalCyclesCompleteAfter < LoadBufferSize){ // Need to readjust > 0) {
      
      IssueCycle = EarliestDispatchCycle;
      
    }
    else {
#ifdef DEBUG_OOO_BUFFERS
      
      DEBUG (dbgs () << "Find  in DispatchToLoadBufferQueueTree the largest than or equal to " << EarliestDispatchCycle +
             1 << "\n");
#endif
      ComplexTree < uint64_t > *Node = DispatchToLoadBufferQueueTree;
      //PrintDispatchToLoadBufferTree();
      while (true) {
        // This is the mechanism used in the original algorithm to delete the host
        // node,  decrementing the last_record attribute of the host node, and
        // the size attribute of all parents nodes.
        // Node->size = Node->size-1;
        if (EarliestDispatchCycle + 1 < Node->key) {
          
          if (Node->left == NULL)
            break;
          if (Node->left->key < EarliestDispatchCycle + 1) {
            break;
          }
          Node = Node->left;
        }
        else {
          if (EarliestDispatchCycle + 1 > Node->key) {
            if (Node->right == NULL)
              break;
            Node = Node->right;
          }
          else {		// Last = Node->key, i.e., Node is the host node
            break;
          }
        }
      }
      
      IssueCycle = Node->key;
#ifdef DEBUG_OOO_BUFFERS
      
      DEBUG (dbgs () << "IssueCycle " << IssueCycle << "\n");
#endif
      PrintDispatchToLoadBufferTree();
      
      //Get the closest larger than or equal to EarliestaDispatchCycle
      
      SimpleTree < uint64_t > *TmpNode = LoadBufferCompletionCyclesTree;
      
      while (true) {
        // This is the mechanism used in the original algorithm to delete the host
        // node,  decrementing the last_record attribute of the host node, and
        // the size attribute of all parents nodes.
        // Node->size = Node->size-1;
        if (EarliestDispatchCycle + 1 < TmpNode->key) {
          
          if (TmpNode->left == NULL)
            break;
          if (TmpNode->left->key < EarliestDispatchCycle + 1) {
            break;
          }
          TmpNode = TmpNode->left;
        }
        else {
          if (EarliestDispatchCycle + 1 > TmpNode->key) {
            if (TmpNode->right == NULL)
              break;
            TmpNode = TmpNode->right;
          }
          else {		// Last = Node->key, i.e., Node is the host node
            break;
          }
        }
      }
      if (TmpNode->key >= EarliestDispatchCycle + 1) {
        IssueCycle = min (TmpNode->key, IssueCycle);
      }
    }
    
    return IssueCycle;
  }
}





uint64_t DynamicAnalysis::FindIssueCycleWhenStoreBufferIsFull ()
{
  
  size_t
  BufferSize = DispatchToStoreBufferQueue.size ();
  
  if (BufferSize == 0) {
    return GetMinCompletionCycleStoreBuffer ();
  }
  else {
    if (BufferSize >= (unsigned) StoreBufferSize) {
      uint64_t
      EarliestCompletion = DispatchToStoreBufferQueue.back ().CompletionCycle;
      for (vector < InstructionDispatchInfo >::iterator it = DispatchToStoreBufferQueue.end () - 1;
           it >= DispatchToStoreBufferQueue.end () - StoreBufferSize; --it) {
        if ((*it).CompletionCycle < EarliestCompletion) {
          EarliestCompletion = (*it).CompletionCycle;
        }
      }
      return EarliestCompletion;
    }
    else {
      sort (StoreBufferCompletionCycles.begin (), StoreBufferCompletionCycles.end ());
      return StoreBufferCompletionCycles[BufferSize];
    }
  }
}



void
DynamicAnalysis::PrintReorderBuffer ()
{
  
  DEBUG (dbgs () << "Reorder Buffer:\n");
  for (unsigned i = 0; i < ReorderBufferCompletionCycles.size (); i++) {
    DEBUG (dbgs () << ReorderBufferCompletionCycles[i] << " ");
  }
  DEBUG (dbgs () << "\n");
}



void
DynamicAnalysis::PrintReservationStation ()
{
  
  DEBUG (dbgs () << "Reservation Station:\n");
  for (unsigned i = 0; i < ReservationStationIssueCycles.size (); i++) {
    DEBUG (dbgs () << ReservationStationIssueCycles[i] << " ");
  }
  DEBUG (dbgs () << "\n");
}



void
DynamicAnalysis::PrintLoadBuffer ()
{
  
  DEBUG (dbgs () << "Load Buffer:\n");
  for (unsigned i = 0; i < LoadBufferCompletionCycles.size (); i++) {
    DEBUG (dbgs () << LoadBufferCompletionCycles[i] << " ");
  }
  DEBUG (dbgs () << "\n");
}

void
DynamicAnalysis::PrintLoadBufferTreeRecursive (SimpleTree < uint64_t > *p)
{
  
  if (p != NULL) {
    if (p->left)
      PrintLoadBufferTreeRecursive (p->left);
    if (p->right)
      PrintLoadBufferTreeRecursive (p->right);
    for(unsigned i = 0; i< p->duplicates;i++)
      
      DEBUG (dbgs () << " " << p->key);
  }
  else
    return;
}


void
DynamicAnalysis::PrintDispatchToLoadBufferTreeRecursive (ComplexTree < uint64_t > *p, bool key)
{
  if (p != NULL) {
    if (p->left)
      PrintDispatchToLoadBufferTreeRecursive (p->left, key);
    if (p->right)
      PrintDispatchToLoadBufferTreeRecursive (p->right, key);
    if (key) {
      for(std::vector<uint64_t>::iterator it = p->IssueCycles.begin(); it != p->IssueCycles.end(); ++it) {
        DEBUG (dbgs () << " " << p->key);
        
      }
    }
    else{
      for(std::vector<uint64_t>::iterator it = p->IssueCycles.begin(); it != p->IssueCycles.end(); ++it) {
        DEBUG (dbgs () << " " << *it);
        
      }
    }
  }
  else
    return;
  
}



void
DynamicAnalysis::PrintLoadBufferTree ()
{
  
  DEBUG (dbgs () << "Load Buffer Tree:\n");
  PrintLoadBufferTreeRecursive (LoadBufferCompletionCyclesTree);
  DEBUG (dbgs () << "\n");
  DEBUG (dbgs () << "Size of Load Buffer Tree: "<< node_size(LoadBufferCompletionCyclesTree)<<"\n");
  
  
}



void
DynamicAnalysis::PrintStoreBuffer ()
{
  
  DEBUG (dbgs () << "Store Buffer:\n");
  for (unsigned i = 0; i < StoreBufferCompletionCycles.size (); i++) {
    DEBUG (dbgs () << StoreBufferCompletionCycles[i] << " ");
  }
  DEBUG (dbgs () << "\n");
}



void
DynamicAnalysis::PrintLineFillBuffer ()
{
  
  DEBUG (dbgs () << "Line Fill Buffer:\n");
  for (unsigned i = 0; i < LineFillBufferCompletionCycles.size (); i++) {
    DEBUG (dbgs () << LineFillBufferCompletionCycles[i] << " ");
  }
  DEBUG (dbgs () << "\n");
}



void
DynamicAnalysis::PrintDispatchToLoadBuffer ()
{
  
  DEBUG (dbgs () << "Dispatch to Load Buffer Issue Cycles:\n");
  for (unsigned i = 0; i < DispatchToLoadBufferQueue.size (); i++) {
    DEBUG (dbgs () << DispatchToLoadBufferQueue[i].IssueCycle << " ");
  }
  DEBUG (dbgs () << "\n");
  DEBUG (dbgs () << "Dispatch to Load Buffer Completion Cycles:\n");
  for (unsigned i = 0; i < DispatchToLoadBufferQueue.size (); i++) {
    DEBUG (dbgs () << DispatchToLoadBufferQueue[i].CompletionCycle << " ");
  }
  DEBUG (dbgs () << "\n");
}


void
DynamicAnalysis::PrintDispatchToLoadBufferTree ()
{

  DEBUG (dbgs () << "Dispatch to Load Buffer Issue Cycles:\n");
  PrintDispatchToLoadBufferTreeRecursive (DispatchToLoadBufferQueueTree, false);
  DEBUG (dbgs () << "\n");
  DEBUG (dbgs () << "Dispatch to Load Buffer Completion Cycles:\n");
  PrintDispatchToLoadBufferTreeRecursive (DispatchToLoadBufferQueueTree, true);
  DEBUG (dbgs () << "\n");
  
}


void
DynamicAnalysis::PrintDispatchToStoreBuffer ()
{
  
  DEBUG (dbgs () << "Dispatch to Store Buffer Issue Cycles:\n");
  for (unsigned i = 0; i < DispatchToStoreBufferQueue.size (); i++) {
    DEBUG (dbgs () << DispatchToStoreBufferQueue[i].IssueCycle << " ");
  }
  DEBUG (dbgs () << "\n");
  
  DEBUG (dbgs () << "Dispatch to Store Buffer Completion Cycles:\n");
  for (unsigned i = 0; i < DispatchToStoreBufferQueue.size (); i++) {
    DEBUG (dbgs () << DispatchToStoreBufferQueue[i].CompletionCycle << " ");
  }
  DEBUG (dbgs () << "\n");
}



void
DynamicAnalysis::PrintDispatchToLineFillBuffer ()
{
  
  DEBUG (dbgs () << "Dispatch to Line Fill Buffer Issue Cycles:\n");
  for (unsigned i = 0; i < DispatchToLineFillBufferQueue.size (); i++) {
    DEBUG (dbgs () << DispatchToLineFillBufferQueue[i].IssueCycle << " ");
  }
  DEBUG (dbgs () << "\n");
  
  DEBUG (dbgs () << "Dispatch to Line Fill Buffer Completion Cycles:\n");
  for (unsigned i = 0; i < DispatchToLineFillBufferQueue.size (); i++) {
    DEBUG (dbgs () << DispatchToLineFillBufferQueue[i].CompletionCycle << " ");
  }
  DEBUG (dbgs () << "\n");
}



void
DynamicAnalysis::IncreaseInstructionFetchCycle (bool EmptyBuffers)
{
  
#ifndef EFF_TBV
  unsigned TreeChunk = 0;
#endif
  bool OOOBufferFull = false;
  
  uint64_t OriginalInstructionFetchCycle = InstructionFetchCycle;
#ifdef DEBUG_GENERIC
  DEBUG (dbgs () << "_____________________ InstructionFetchCycle " << InstructionFetchCycle << "_____________________\n");
#endif
#ifdef MOO_BUFFERS
#ifdef DEBUG_OOO_BUFFERS
  PrintReservationStation ();
  PrintReorderBuffer ();
  PrintStoreBuffer ();
  if(SmallBuffers)
    PrintLoadBuffer();
  else
    PrintLoadBufferTree ();
  
  PrintLineFillBuffer ();
  if(SmallBuffers)
    PrintDispatchToLoadBuffer();
  else
    PrintDispatchToLoadBufferTree ();
  
  PrintDispatchToStoreBuffer ();
  
  PrintDispatchToLineFillBuffer ();
#endif
#endif
  
  
  // Remove from Reservation Stations elements issued at fetch cycle
  if (ReservationStationSize > 0)
    RemoveFromReservationStation (InstructionFetchCycle);
  
  if (ReorderBufferSize > 0)
    RemoveFromReorderBuffer (InstructionFetchCycle);
  
  
  
#ifdef MOO_BUFFERS
  //Remove from Load, Store and Fill Line Buffers elements completed at issue cycle
  
  if(SmallBuffers){
    RemoveFromLoadBuffer(InstructionFetchCycle);
    RemoveFromDispatchToLoadBufferQueue(InstructionFetchCycle);
  }else
    RemoveFromLoadBufferTree (InstructionFetchCycle);
  
  RemoveFromStoreBuffer (InstructionFetchCycle);
  RemoveFromLineFillBuffer (InstructionFetchCycle);
  
  RemoveFromDispatchToStoreBufferQueue (InstructionFetchCycle);
  RemoveFromDispatchToLineFillBufferQueue (InstructionFetchCycle);
  // Insert into LB, SB and LFB the instructions from the dispatch queue.
  if(SmallBuffers)
    DispatchToLoadBuffer(InstructionFetchCycle);
  else
    DispatchToLoadBufferTree (InstructionFetchCycle);
  
  DispatchToStoreBuffer (InstructionFetchCycle);
  DispatchToLineFillBuffer (InstructionFetchCycle);
#endif
  
#ifdef MOO_BUFFERS
#ifdef DEBUG_OOO_BUFFERS
  PrintReservationStation ();
  PrintReorderBuffer ();
  PrintStoreBuffer ();
  if(SmallBuffers)
    PrintLoadBuffer();
  else
    PrintLoadBufferTree ();
  
  PrintLineFillBuffer ();
  PrintDispatchToStoreBuffer ();
  if(SmallBuffers)
    PrintDispatchToLoadBuffer();
  else
    PrintDispatchToLoadBufferTree ();
  
  PrintDispatchToLineFillBuffer ();
#endif
#endif
  // If Reservation station is full
  if(ReservationStationIssueCycles.size () == (unsigned) ReservationStationSize && ReservationStationSize != 0){
    
    // Advance InstructionFetchCyle until min issue cycle
    OOOBufferFull = true;
    uint64_t CurrentInstructionFetchCycle = InstructionFetchCycle;
    InstructionFetchCycle = GetMinIssueCycleReservationStation ();
    
    if (InstructionFetchCycle > CurrentInstructionFetchCycle + 1)
      FirstNonEmptyLevel[RS_STALL] =
      (FirstNonEmptyLevel[RS_STALL] == 0) ? CurrentInstructionFetchCycle + 1 : FirstNonEmptyLevel[RS_STALL];
    
    
    for (uint64_t i = CurrentInstructionFetchCycle + 1; i < InstructionFetchCycle; i++) {
      
      //TODO: FIX!
#ifdef EFF_TBV
      GetTreeChunk (i,RS_STALL );
      FullOccupancyCyclesTree[RS_STALL].insert_node ( i);
#else
      
      TreeChunk = GetTreeChunk (i);
      FullOccupancyCyclesTree[TreeChunk].insert_node (i, RS_STALL);
#endif
      
      
      
      
      
#ifdef SOURCE_CODE_ANALYSIS
      FullOccupancyCyclesTree[TreeChunk].insert_source_code_line (i, SourceCodeLine, RS_STALL);
#endif
      InstructionsCountExtended[RS_STALL]++;
      InstructionsLastIssueCycle[RS_STALL] = i;
    }
    
#ifdef DEBUG_OOO_BUFFERS
    DEBUG (dbgs () << "Updating InstructionFetchCycle to " << InstructionFetchCycle << " \n");
#endif
  }
  if (ReorderBufferCompletionCycles.size () == ReorderBufferSize && ReorderBufferSize > 0) {
    
    //Advance InstructionFetchCycle to the head of the buffer
    OOOBufferFull = true;
    uint64_t CurrentInstructionFetchCycle = InstructionFetchCycle;
    InstructionFetchCycle = max (InstructionFetchCycle, ReorderBufferCompletionCycles.front ());
    
    if (InstructionFetchCycle > CurrentInstructionFetchCycle + 1) {
      FirstNonEmptyLevel[ROB_STALL] =
      (FirstNonEmptyLevel[ROB_STALL] == 0) ? CurrentInstructionFetchCycle + 1 : FirstNonEmptyLevel[ROB_STALL];
    }
    
    
    for (uint64_t i = CurrentInstructionFetchCycle + 1; i < InstructionFetchCycle; i++) {
      // Get the node, if any, corresponding to this issue cycle.
#ifdef EFF_TBV
      GetTreeChunk (i,ROB_STALL);
      FullOccupancyCyclesTree[ROB_STALL].insert_node (i);
#else
      TreeChunk = GetTreeChunk (i);
      FullOccupancyCyclesTree[TreeChunk].insert_node (i, ROB_STALL);
#endif
      
#ifdef SOURCE_CODE_ANALYSIS
      FullOccupancyCyclesTree[TreeChunk].insert_source_code_line (i, SourceCodeLine, ROB_STALL);
#endif
      InstructionsCountExtended[ROB_STALL]++;
      InstructionsLastIssueCycle[ROB_STALL] = i;
    }
#ifdef DEBUG_OOO_BUFFERS
    DEBUG (dbgs () << "Updating InstructionFetchCycle to " << InstructionFetchCycle << " \n");
#endif
  }
  if (OOOBufferFull == true) {
    // Remove from Reservation Stations elements issued at fetch cycle
#ifdef DEBUG_OOO_BUFFERS
    PrintReservationStation ();
    PrintReorderBuffer ();
#ifdef MOO_BUFFERS
    PrintStoreBuffer ();
    if(SmallBuffers)
      PrintLoadBuffer();
    else
      PrintLoadBufferTree ();
    
    PrintLineFillBuffer ();
    PrintDispatchToStoreBuffer ();
    if(SmallBuffers)
      PrintDispatchToLoadBuffer();
    else
      PrintDispatchToLoadBufferTree ();
    
    PrintDispatchToLineFillBuffer ();
#endif
#endif
    
    RemoveFromReservationStation (InstructionFetchCycle);
    RemoveFromReorderBuffer (InstructionFetchCycle);
    
#ifdef MOO_BUFFERS
    //Remove from Load, Store and Fill Line Buffers elements completed at issue cycle
    if(SmallBuffers){
      RemoveFromLoadBuffer(InstructionFetchCycle);
      RemoveFromDispatchToLoadBufferQueue(InstructionFetchCycle);
    }else
      RemoveFromLoadBufferTree (InstructionFetchCycle);
    
    
    RemoveFromStoreBuffer (InstructionFetchCycle);
    RemoveFromLineFillBuffer (InstructionFetchCycle);
    
    RemoveFromDispatchToStoreBufferQueue (InstructionFetchCycle);
    RemoveFromDispatchToLineFillBufferQueue (InstructionFetchCycle);
    // Insert into LB, SB and LFB the instructions from the dispatch queue.
    if(SmallBuffers)
      DispatchToLoadBuffer(InstructionFetchCycle);
    else
      DispatchToLoadBufferTree (InstructionFetchCycle);
    
    DispatchToStoreBuffer (InstructionFetchCycle);
    DispatchToLineFillBuffer (InstructionFetchCycle);
#endif
    
#ifdef DEBUG_OOO_BUFFERS
    PrintReservationStation ();
    PrintReorderBuffer ();
#ifdef MOO_BUFFERS
    PrintStoreBuffer ();
    if(SmallBuffers)
      PrintLoadBuffer();
    else
      PrintLoadBufferTree ();
    
    PrintLineFillBuffer ();
    PrintDispatchToStoreBuffer ();
    if(SmallBuffers)
      PrintDispatchToLoadBuffer();
    else
      PrintDispatchToLoadBufferTree ();
    PrintDispatchToLineFillBuffer ();
#endif
#endif
  }
  
  // When we are at this point, either we have removed from RS or ROB the
  // instructions issued at this cycle, and they left some empty slots
  // so that the buffers are not full anymore, or we have advanced
  // InstructionFetchCycle to the cycle at which any of the buffers
  // gets empty. In this case, we also have to set Remaining instructions
  // to fetch to Fetchbanwidth, because we have modified fetch cycle
  // and we start fetching again.
  if (OOOBufferFull == true) {
    RemainingInstructionsFetch = InstructionFetchBandwidth;
    
  }
  
  if (EmptyBuffers == true) {
    InstructionFetchCycle++;
  }
  else {
    if (RemainingInstructionsFetch == 0 && InstructionFetchBandwidth != INF) {
      InstructionFetchCycle++;
      RemainingInstructionsFetch = InstructionFetchBandwidth;
      
      
    }
  }
  

  
  if (OriginalInstructionFetchCycle != InstructionFetchCycle) {
    uint64_t CyclesIncrease = (InstructionFetchCycle-OriginalInstructionFetchCycle);
    BuffersOccupancy[RS_STALL - RS_STALL] += (ReservationStationIssueCycles.size() * CyclesIncrease);
    BuffersOccupancy[ROB_STALL - RS_STALL] += (ReorderBufferCompletionCycles.size()* CyclesIncrease);
    if(SmallBuffers){
      BuffersOccupancy[LB_STALL-RS_STALL] += (LoadBufferCompletionCycles.size()*CyclesIncrease);
      if(LoadBufferCompletionCycles.size() > LoadBufferSize)
        report_fatal_error ("Buffer overflow");
      
    }else{
      BuffersOccupancy[LB_STALL - RS_STALL] += (node_size (LoadBufferCompletionCyclesTree)*CyclesIncrease);
      if( node_size(LoadBufferCompletionCyclesTree) > LoadBufferSize){
        dbgs() <<  "node_size (LoadBufferCompletionCyclesTree) " << node_size (LoadBufferCompletionCyclesTree) << "\n";
        dbgs() <<  "LoadBufferSize " <<LoadBufferSize << "\n";
        report_fatal_error ("Buffer overflow");
      }
    }
    BuffersOccupancy[SB_STALL - RS_STALL] += (StoreBufferCompletionCycles.size()*CyclesIncrease);
    BuffersOccupancy[LFB_STALL - RS_STALL] += (LineFillBufferCompletionCycles.size()*CyclesIncrease);
    
    uint64_t PrevInstructionFetchCycle = InstructionFetchCycle - 1;
    if (DispatchToLineFillBufferQueue.empty () == false) {
      if (InstructionsCountExtended[LFB_STALL] == 0)
        FirstIssue[LFB_STALL] = true;
      if (FirstIssue[LFB_STALL] == true) {
        FirstNonEmptyLevel[LFB_STALL] = PrevInstructionFetchCycle;
        FirstIssue[LFB_STALL] = false;
      }
      
      InstructionsLastIssueCycle[LFB_STALL] = PrevInstructionFetchCycle;

#ifdef EFF_TBV
      GetTreeChunk (PrevInstructionFetchCycle, LFB_STALL);
      FullOccupancyCyclesTree[LFB_STALL].insert_node (PrevInstructionFetchCycle);
#else
      TreeChunk = GetTreeChunk (PrevInstructionFetchCycle);
      FullOccupancyCyclesTree[TreeChunk].insert_node (PrevInstructionFetchCycle, LFB_STALL);
#endif
      // We do it when an instruction is inserted. Otherwise, SourceCodeLine has the value
      // of the last instruction analyzed from the instruction fetch window, which
      // might not be the instruction that was stalled.
      

      InstructionsCountExtended[LFB_STALL]++;
    }
    
    bool BufferNonEmpty=false;
    if(SmallBuffers){
      if (DispatchToLoadBufferQueue.empty() == false)
        BufferNonEmpty = true;
    }else{
      if (node_size (DispatchToLoadBufferQueueTree) != 0)
        BufferNonEmpty = true;
    }
    // if (node_size (DispatchToLoadBufferQueueTree) != 0) {
    //if (DispatchToLoadBufferQueue.empty() == false) {
    if(BufferNonEmpty){
      if (InstructionsCountExtended[LB_STALL] == 0)
        FirstIssue[LB_STALL] = true;
      if (FirstIssue[LB_STALL] == true) {
        FirstNonEmptyLevel[LB_STALL] = PrevInstructionFetchCycle;
        FirstIssue[LB_STALL] = false;
      }
      InstructionsLastIssueCycle[LB_STALL] = PrevInstructionFetchCycle;

#ifdef EFF_TBV
      GetTreeChunk (PrevInstructionFetchCycle,LB_STALL);
      FullOccupancyCyclesTree[LB_STALL].insert_node (PrevInstructionFetchCycle);
#else
      TreeChunk = GetTreeChunk (PrevInstructionFetchCycle);
      FullOccupancyCyclesTree[TreeChunk].insert_node (PrevInstructionFetchCycle, LB_STALL);
#endif
#ifdef DEBUG_OOO_BUFFERS
      
      DEBUG (dbgs () << "LB_STALL in cycle " << PrevInstructionFetchCycle << "\n");
#endif
      

      InstructionsCountExtended[LB_STALL]++;
    }
    
    if (DispatchToStoreBufferQueue.empty () == false) {
      if (InstructionsCountExtended[SB_STALL] == 0)
        FirstIssue[SB_STALL] = true;
      if (FirstIssue[SB_STALL] == true) {
        FirstNonEmptyLevel[SB_STALL] = PrevInstructionFetchCycle;
        FirstIssue[SB_STALL] = false;
      }
      
      InstructionsLastIssueCycle[SB_STALL] = PrevInstructionFetchCycle;
#ifdef DEBUG_SOURCE_CODE_ANALYSIS
      DEBUG (dbgs () << "Inserting SB_STALL when instruction fetch cycle changes for cycle " << PrevInstructionFetchCycle <<
             "\n");
#endif
    
#ifdef EFF_TBV
      GetTreeChunk (PrevInstructionFetchCycle, SB_STALL);
      FullOccupancyCyclesTree[SB_STALL].insert_node (PrevInstructionFetchCycle);
#else
      TreeChunk = GetTreeChunk (PrevInstructionFetchCycle);
      FullOccupancyCyclesTree[TreeChunk].insert_node (PrevInstructionFetchCycle, SB_STALL);
#endif
      InstructionsCountExtended[SB_STALL]++;
    }
  }
  
  
  
  
}


//===----------------------------------------------------------------------===//
//          Main method for analysis of the instruction properties
//                    (dependences, reuse, etc.)
//===----------------------------------------------------------------------===//


// Handling instructions dependences with def-use chains.
// Whenever there is a def, and we know the issue cycle (IssueCycle )of the def,
// update all the uses of that definition with IssueCycle+1.
// The issue cycle of an instruction is hence the max of the issue cycles of its
// operands, but the issue cycle of its operands does not have to be determined,
// already contains the right value because they are uses of a previous definition.

#ifdef INTERPRETER
void
DynamicAnalysis::analyzeInstruction (Instruction & I, ExecutionContext & SF, GenericValue * visitResult)
{
  unsigned OpCode = I.getOpcode ();
  unsigned forceAnalyze = false;
  unsigned Line = 0;
#else
  void
  DynamicAnalysis::analyzeInstruction (Instruction & I, unsigned OpCode, uint64_t addr, unsigned Line, bool forceAnalyze)
  {
#endif
    
    
    
    int k = 0;
    int Distance;
    int RegisterStackDistance = -1;
    int NextCacheLineExtendedInstructionType;
    int InstructionType = getInstructionType (I);
    
    
    CacheLineInfo Info;
    uint64_t CacheLine = 0;
    uint64_t MemoryAddress = 0;
    uint64_t LoadCacheLine;
    uint64_t StoreCacheLine;
    uint64_t NextCacheLine;
    uint64_t NextCacheLineIssueCycle;
    uint64_t InstructionIssueCycle, OriginalInstructionIssueCycle, LastAccess;
    uint64_t InstructionIssueFetchCycle =
    0, InstructionIssueLoadBufferAvailable =
    0, InstructionIssueLineFillBufferAvailable =
    0, InstructionIssueStoreBufferAvailable =
    0, InstructionIssueAGUAvailable = 0, InstructionIssueDataDeps =
    0, InstructionIssueCacheLineAvailable = 0, InstructionIssueMemoryModel =
    0, InstructionIssueStoreAGUAvailable =
    0, InstructionIssueLoadAGUAvailable = 0, InstructionIssuePortAvailable = 0, InstructionIssueThroughputAvailable =
    0, InstructionIssueCycleFirstTimeAvailable = 0;
    uint Latency = 0;
    uint LatencyPrefetch = 0;
    
    //Aux vars for handling arguments of a call instruction
    CallSite CS;
    Function *F;
    std::vector < Value * >ArgVals;
    unsigned NumArgs;
    unsigned ExtendedInstructionType = InstructionType;
    unsigned ExecutionResource = 0;
    unsigned NElementsVector = 1;
    
    bool IsVectorInstruction = false;
    bool isLoad = true;
    bool isRegisterSpill = false;
    
    unsigned Port = 0;
    
    // Reset IssuePorts
    IssuePorts = vector < unsigned >();
    
    TotalInstructions++;
    vector < uint64_t > emptyVector;
    
    
    
#ifdef MICROSCHEDULING
    if (dyn_cast < TerminatorInst > (&I)) {
      DEBUG (dbgs () << "New Basic Block\n");
      BasicBlockLookAhead++;
      if (BasicBlockLookAhead == 1) {
        BasicBlockLookAhead = 0;
        BasicBlockBarrier = 0;
        for (j = 0; j < 6; j++) {
          BasicBlockBarrier = max ((unsigned long) BasicBlockBarrier, DAGLevelsOccupancy[j].size ());
        }
        DEBUG (dbgs () << "BasicBlockBarrier " << BasicBlockBarrier << "\n");
      }
    }
#endif
    
    //====================== WARM CACHE ANALYSIS - RECORD ONLY MEMORY ACCESSES =========//
    if (WarmCache && rep == 0) {
      
      
      if (InstructionType >= 0 || forceAnalyze == true) {
        
        switch (OpCode) {
          case Instruction::Load:
          {
#ifdef INTERPRETER
            //Transform visitResult to uint64_t
            SmallString < 128 > StrVal;
            raw_svector_ostream OS (StrVal);
            OS << visitResult;
            MemoryAddress = strtol (OS.str ().str ().c_str (), NULL, 16);
#else
            MemoryAddress = addr;
#endif
            
            CacheLine = MemoryAddress >> BitsPerCacheLine;
            Info = getCacheLineInfo (CacheLine);
            
            
            //Code for reuse calculation
            RegisterStackDistance = RegisterStackReuseDistance(MemoryAddress);
            if (RegisterStackDistance < 0){
              Distance = ReuseDistance (Info.LastAccess, TotalInstructions, CacheLine);
              Info.LastAccess = TotalInstructions;
              
              insertCacheLineLastAccess (CacheLine, Info.LastAccess);
            }
            //   insertMemoryAddressIssueCycle(MemoryAddress, TotalInstructions);
            //   updateReuseDistanceDistribution(Distance, InstructionIssueCycle);
            
            ExtendedInstructionType = GetExtendedInstructionType (Instruction::Load, Distance, RegisterStackDistance);
          }
            break;
          case Instruction::Store:
          {
#ifdef INTERPRETER
            SmallString < 128 > StrVal;
            raw_svector_ostream OS (StrVal);
            OS << visitResult;
            MemoryAddress = strtol (OS.str ().str ().c_str (), NULL, 16);
#else
            MemoryAddress = addr;
#endif
            CacheLine = MemoryAddress >> BitsPerCacheLine;
            Info = getCacheLineInfo (CacheLine);
            
            RegisterStackDistance = RegisterStackReuseDistance(MemoryAddress);
            if (RegisterStackDistance < 0){
              Distance = ReuseDistance (Info.LastAccess, TotalInstructions, CacheLine);
              
              
              Info.LastAccess = TotalInstructions;
              insertCacheLineLastAccess (CacheLine, Info.LastAccess);
            }
            //     insertMemoryAddressIssueCycle(MemoryAddress, TotalInstructions);
            //  updateReuseDistanceDistribution(Distance, InstructionIssueCycle);
            ExtendedInstructionType = GetExtendedInstructionType (Instruction::Store, Distance,RegisterStackDistance);
          }
            break;
            
          default:
            break;
        }
        
        // ============================ SPATIAL PREFETCHER ==============================
        
        if (SpatialPrefetcher && (OpCode == Instruction::Load || OpCode == Instruction::Store)
            && (ExtendedInstructionType > PrefetchDispatch && !(ExecutionUnit[ExtendedInstructionType] == PrefetchLevel))
           
            ) {
          
          NextCacheLine = CacheLine + 1;
          
          //Get reuse distance of NextCacheLine
          Info = getCacheLineInfo (NextCacheLine);
          Distance = ReuseDistance (Info.LastAccess, TotalInstructions, NextCacheLine, true);
          NextCacheLineExtendedInstructionType = GetMemoryInstructionType (Distance, MemoryAddress, isLoad);
          
          ExecutionResource = ExecutionUnit[NextCacheLineExtendedInstructionType];
          
#ifdef DEBUG_PREFETCHER
          DEBUG (dbgs () << "CacheLine " << CacheLine << "\n");
          DEBUG (dbgs () << "NextCacheLine " << NextCacheLine << "\n");
          DEBUG (dbgs () << "Execution Resource  " << GetResourceName (ExecutionResource) << "\n");
          DEBUG (dbgs () << "PrefetchTarget  " << PrefetchTarget << "\n");
          DEBUG (dbgs () << "PrefetchLevel  " << PrefetchLevel << "\n");
          
#endif
          // Only bring data from memory to the die, not for example, from LLC to L2
          // if (ExecutionResource == MEM_LOAD_CHANNEL || ExecutionResource == MEM_STORE_CHANNEL) {
          if (ExecutionResource > PrefetchTarget && ExecutionResource >= PrefetchDestination) {
#ifdef DEBUG_PREFETCHER
            DEBUG (dbgs () << "Prefetching next cache line which is in " << GetResourceName (ExecutionResource) << "\n");
#endif
            Info.LastAccess = TotalInstructions;
            insertCacheLineLastAccess (NextCacheLine, Info.LastAccess);
          }
        }
      }
      
      //====================== END OF WARM CACHE ANALYSIS  =========//
    }
    else {
      
      
      if (InstructionType >= 0 || forceAnalyze == true) {
        
        DEBUG (dbgs () << I << " (" << &I << ")\n");
        // Determine instruction width
        int NumOperands = I.getNumOperands ();
        
        if (NumOperands > 0) {
          int OperandPosition = (OpCode == Instruction::Store) ? 1 : 0;
          Type *Ty = I.getOperand (OperandPosition)->getType ();
          // If load/store, the operand is a pointer
          if (PointerType * PT = dyn_cast < PointerType > (Ty)) {
            
            
            if (PT->getElementType ()->getTypeID () == Type::VectorTyID) {
              DEBUG (dbgs () << "The type of the operand is a vector\n");
              VectorCode = true;
              IsVectorInstruction = true;
              NElementsVector = PT->getElementType ()->getVectorNumElements ();
            }
          }
          
          
          // If arithmetic instruction, we can get the vector type directly
          
          if (Ty->getTypeID () == Type::VectorTyID) {
            VectorCode = true;
            IsVectorInstruction = true;
            NElementsVector = Ty->getVectorNumElements ();
            //TODO: Dont' make it dependt on the opcode, but on the node type
            if (GetExtendedInstructionType (OpCode) == FP_SHUFFLE_NODE || GetExtendedInstructionType (OpCode) == FP_MOV_NODE
                || GetExtendedInstructionType (OpCode) == FP_BLEND_NODE) {
              NElementsVector = 1;
            }
            
          }
        }
        
        if (IsVectorInstruction) {
          DEBUG (dbgs () << "Vector instruction of width " << NElementsVector << "\n");
          if (ShareThroughputAmongPorts[ExecutionUnit[GetExtendedInstructionType(OpCode)]]==false)
            report_fatal_error("Executing vector instruction without sharing ports. It may be correct, but double check that this is the desired behaviour");
        }
        
        //      }
        
        
        
        
        //================= Update Fetch Cycle, remove insts from buffers =========//
        // EVERY INSTRUCTION IN THE RESERVATION STATION IS ALSO IN THE REORDER BUFFER
        
        //    if (InstructionType >= 0 || forceAnalyze == true) {
        SourceCodeLine = Line;
        /*
         #ifdef SOURCE_CODE_ANALYSIS
         // Get line number
         
         
         DebugLoc Loc = I.getDebugLoc(); // Here I is an LLVM instruction
         unsigned Line = Loc.getLine();
         
         
         if(Line != 0){
         dbgs()<<  I<< " ("<< &I <<")\n";
         dbgs() << "Line "<<Line <<"\n";
         }
         
         if (MDNode *N = I.getMetadata("dbg")) {  // Here I is an LLVM instruction
         
         DILocation Loc(N);                      // DILocation is in DebugInfo.h
         SourceCodeLine = Loc.getLineNumber();
         if(SourceCodeLine != 0){
         dbgs()<<  I<< " ("<< &I <<")\n";
         dbgs() << "SourceCodeLine "<<SourceCodeLine <<"\n";
         }
         }else{
         // Cannot get metadata of the instruction
         }
         //  if (SourceCodeLine == 0) {
         //   report_fatal_error("Source code analysis requires the applicaiton to be compiled with -g flag");
         // }
         
         #endif
         */
        
        //   if (ReservationStationIssueCycles.size() == (unsigned)ReservationStationSize) {
        if (RemainingInstructionsFetch == 0 ||	/*RemainingInstructionsFetch == INF|| */
            (ReorderBufferCompletionCycles.size () == (unsigned) ReorderBufferSize && ReorderBufferSize != 0)
            || (ReservationStationIssueCycles.size () == (unsigned) ReservationStationSize && ReservationStationSize != 0)) {
          
          IncreaseInstructionFetchCycle ();
          
        }
      }
      
      
      
      
      //==================== Handle special cases ===============================//
      switch (OpCode) {
          // Dependences through PHI nodes
        case Instruction::Br:
        case Instruction::IndirectBr:
        case Instruction::Switch:
        {
#ifdef DEBUG_PHI_NODE
          DEBUG (dbgs () << "Loop over all of the PHI nodes in the current block\n");
#endif
#ifdef INTERPRETER
          // Loop over all of the PHI nodes in the current block, reading their inputs.
          SF.CurInst = SF.CurBB->begin ();
          for (unsigned i = 0; PHINode * PN = dyn_cast < PHINode > (SF.CurInst); ++SF.CurInst, ++i)
                   {
            //Value *Predecesor = PN->getIncomingValue(PN->getBasicBlockIndex(I.getParent()));
            // The PHI node was a use of its predecessor. Hence, its entry in the map
            //contains the correct value of the InstructionIssueCycle
            // InstructionIssueCycle = getInstructionValueIssueCycle(Predecesor); -> WRONG!!
            InstructionIssueCycle = max (max (InstructionFetchCycle, BasicBlockBarrier), getInstructionValueIssueCycle (PN));
#ifdef DEBUG_PHI_NODE
            DEBUG (dbgs () << "PHI Node " << PN << "\n");
            DEBUG (dbgs () << "InstructionValueIssueCycle of PHI Node " << InstructionIssueCycle << "\n");
#endif
            
#ifdef INTERPRETER
            // Iterate through the uses of the PHI node
            for (Value::use_iterator i = PN->use_begin (), ie = PN->use_end (); i != ie; ++i) {
#ifdef DEBUG_PHI_NODE
              DEBUG (dbgs () << "Use of the PHI node " << *i << "\n");
#endif
              if (dyn_cast < PHINode > (*i)) {
                insertInstructionValueIssueCycle (*i, InstructionIssueCycle, true);
              }
              else {
                insertInstructionValueIssueCycle (*i, InstructionIssueCycle);
              }
              
            }
           
#endif
          }
#endif
          InstructionIssueCycle = max (max (InstructionFetchCycle, BasicBlockBarrier), getInstructionValueIssueCycle (&I));	// This is the branch instrucion
          
          //Iterate over the uses of the generated value
#if LLVM_VERSION_MINOR<6
          for (Value::use_iterator i = I.use_begin (), ie = I.use_end (); i != ie; ++i) {
            insertInstructionValueIssueCycle (*i, InstructionIssueCycle + 1 /*??? */ );
          }
#else
          for (User * U:I.users ()) {
            insertInstructionValueIssueCycle (U, InstructionIssueCycle + 1 /*??? */ );
            
          }
#endif
        }
          
          break;
        case Instruction::PHI:
#ifndef INTERPRETER
          InstructionIssueCycle = max (max (InstructionFetchCycle, BasicBlockBarrier), getInstructionValueIssueCycle (&I));	// This is the branch instrucion
#ifdef DEBUG_PHI_NODE
          DEBUG (dbgs () << "Executing PHI Node. Iterate over it uses.\n");
#endif
          // Iterate through the uses of the PHI node
          //         for (User *U : I.users()) {
          for (Value::use_iterator i = I.use_begin (), ie = I.use_end (); i != ie; ++i) {
            
#ifdef DEBUG_PHI_NODE
            DEBUG (dbgs () << "Use of the PHI node " << U << "\n");
#endif
            if (dyn_cast < PHINode > (*i)) {
              insertInstructionValueIssueCycle (*i, InstructionIssueCycle, true);
              
            }
            else {
              insertInstructionValueIssueCycle (*i, InstructionIssueCycle);
              
            }
            
            
          }
#endif
          break;
          // Dependences through the arguments of a method call
        case Instruction::Call:
          CS = CallSite (&I);
          F = CS.getCalledFunction ();
          // Loop over the arguments of the called function --- From Execution.cpp
          NumArgs = CS.arg_size ();
          ArgVals.reserve (NumArgs);
          for (CallSite::arg_iterator i = CS.arg_begin (), e = CS.arg_end (); i != e; ++i) {
            Value *V = *i;
            ArgVals.push_back (V);
          }
          InstructionIssueCycle = max (max (InstructionFetchCycle, BasicBlockBarrier), getInstructionValueIssueCycle (&I));
          
          if (F->getName ().find ("llvm.x86.avx") != string::npos) {
            dbgs () << "Call to broadcast\n";
            dbgs () << "Iterate through the uses of the broadcast\n";
            dbgs () << I << "\n";
            //#ifdef INTERPRETER
            
#if LLVM_VERSION_MINOR<6
            for (Value::use_iterator i = I.use_begin (), ie = I.use_end (); i != ie; ++i) {
              dbgs () << &i << "\n";
            }
#else
            for (User * U:I.users ()) {
              dbgs () << *i << "\n";
            }
#endif
            
          }
          break;
          
          //-------------------- Memory Dependences -------------------------------//
        case Instruction::Load:
          if (InstructionType >= 0 || forceAnalyze == true) {
            
            isLoad = true;
#ifdef INTERPRETER
            //Transform visitResult to uint64_t
            SmallString < 128 > StrVal;
            raw_svector_ostream OS (StrVal);
            OS << visitResult;
            // OS.str() returns StringRef
            // StringRef.str() returns the contents as a std::string
            // std::string.c_str() converts the string into the const char* required by strtol
            MemoryAddress = strtol (OS.str ().str ().c_str (), NULL, 16);
#else
            MemoryAddress = addr;
#endif
            CacheLine = MemoryAddress >> BitsPerCacheLine;
            
#ifdef DEBUG_MEMORY_TRACES
            DEBUG (dbgs () << "MemoryAddress " << MemoryAddress << "\n");
            DEBUG (dbgs () << "CacheLine " << CacheLine << "\n");
#endif
            
            LoadCacheLine = CacheLine;
            Info = getCacheLineInfo (LoadCacheLine);
            
            
            // Check whether the value is stored in a stack variable
            
            //Code for reuse calculation
            RegisterStackDistance = RegisterStackReuseDistance(MemoryAddress);
            // If we load from L1 or other levels a variable that was allocated in the stack,
            // then that was a register spill.
            if (dyn_cast < AllocaInst > (I.getOperand(0))) {
              DEBUG (dbgs () << "Loading a value from variable allocated in the stack (i.e., a \'register\')\n");
              if (RegisterStackDistance< 0){
                DEBUG (dbgs () << "REGISTER SPILL\n");
                NRegisterSpills++;
                isRegisterSpill = true;
              }
            }
           
            if (RegisterStackDistance< 0){

              Distance = ReuseDistance (Info.LastAccess, TotalInstructions, CacheLine);

              updateReuseDistanceDistribution (Distance, InstructionIssueCycle);
            }
            // Get the new instruction type depending on the reuse distance
            //ExtendedInstructionType = GetMemoryInstructionType(Distance, MemoryAddress);
            ExtendedInstructionType = GetExtendedInstructionType (Instruction::Load, Distance,RegisterStackDistance);
            
#ifdef INT_FP_OPS
            if (ExtendedInstructionType == L1_LOAD_NODE) {
              Q[0] += AccessGranularities[nArithmeticExecutionUnits + 0];
              Q[4] += AccessGranularities[nArithmeticExecutionUnits + 0];
              //  fprintf(stderr, "\tL1\n");
            }
            else if (ExtendedInstructionType == L2_LOAD_NODE) {
              Q[1] += AccessGranularities[nArithmeticExecutionUnits + 2];
              Q[4] += AccessGranularities[nArithmeticExecutionUnits + 2];
              //  fprintf(stderr, "\tL2\n");
            }
            else if (ExtendedInstructionType == L3_LOAD_NODE) {
              Q[2] += AccessGranularities[nArithmeticExecutionUnits + 3];
              Q[4] += AccessGranularities[nArithmeticExecutionUnits + 3];
              //  fprintf(stderr, "\tL3\n");
            }
            else if (ExtendedInstructionType == MEM_LOAD_NODE) {
              Q[3] += AccessGranularities[nArithmeticExecutionUnits + 4];
              Q[4] += AccessGranularities[nArithmeticExecutionUnits + 4];
              //  fprintf(stderr, "\tMem\n");
            }
            else {
              report_fatal_error ("Load Mem op has nowhere to live");
            }
#endif
            
            ExecutionResource = ExecutionUnit[ExtendedInstructionType];
            Latency = ExecutionUnitsLatency[ExecutionResource];
            
            
            
            // UpdateInstructionCount(InstructionType,ExtendedInstructionType, NElementsVector, IsVectorInstruction);
            if(ExtendedInstructionType != REGISTER_LOAD_NODE){
              if (IsVectorInstruction) {
                InstructionsCount[InstructionType] = InstructionsCount[InstructionType] + NElementsVector;
              }
              else
                InstructionsCount[InstructionType]++;
            }
#ifdef DEBUG_REUSE_DISTANCE
            DEBUG (dbgs () << "ExtendedInstructionType " << GetNodeName (ExtendedInstructionType) << "\n");
            DEBUG (dbgs () << "Load latency " << Latency << "\n");
            DEBUG(dbgs() << "RegisterStackDistance " << RegisterStackDistance << "\n");
            DEBUG (dbgs () << "Reuse distance " << Distance << "\n");
            
#endif
            
            InstructionIssueFetchCycle = InstructionFetchCycle;
            
            if (ExtendedInstructionType >= L1_LOAD_NODE){
              InstructionIssueCacheLineAvailable = Info.IssueCycle;
              insertCacheLineLastAccess (LoadCacheLine, TotalInstructions);
            }
#ifdef MOO_BUFFERS
            if(ExtendedInstructionType != REGISTER_LOAD_NODE){
              //Calculate issue cycle depending on buffer Occupancy.
              if (LoadBufferSize > 0) {
                
                bool BufferFull=false;
                if(SmallBuffers){
                  if (LoadBufferCompletionCycles.size() == LoadBufferSize)
                    BufferFull = true;
                }else{
                  if (node_size (LoadBufferCompletionCyclesTree) == LoadBufferSize)
                    BufferFull = true;
                }
                
                               if(BufferFull){
                  if(SmallBuffers)
                    InstructionIssueLoadBufferAvailable = FindIssueCycleWhenLoadBufferIsFull();
                  else
                    InstructionIssueLoadBufferAvailable = FindIssueCycleWhenLoadBufferTreeIsFull ();
                  
                  
                  // If, moreover, the instruction has to go to the LineFillBuffer...
                  if (ExtendedInstructionType >= L2_LOAD_NODE && LineFillBufferSize > 0) {
                    if (LineFillBufferCompletionCycles.size () == (unsigned) LineFillBufferSize) {
                      InstructionIssueLineFillBufferAvailable = FindIssueCycleWhenLineFillBufferIsFull ();
                    }
                  }
                }
                else {		// If the Load Buffer is not fulll...
                  if (ExtendedInstructionType >= L2_LOAD_NODE && LineFillBufferSize > 0) {	// If it has to go to the LFS...
                    
                    if (LineFillBufferCompletionCycles.size () == LineFillBufferSize || !DispatchToLineFillBufferQueue.empty ()) {
                      InstructionIssueLineFillBufferAvailable = FindIssueCycleWhenLineFillBufferIsFull ();
                    }
                    else {		// There is space on both
                      // Do nothing -> Instruction Issue Cycle is not affected by LB or LFB Occupancy
                      //Later, insert in both
                    }
                  }
                  else {		// It does not have to go to LFB...
                    //Do nothing (insert into LoadBuffer later, afte knowing IssueCycle
                    // depending on BW availability. Right not IssueCycle is not affected)
                  }
                }
              }
            }
#endif
            
            // If there is Load buffer, the instruction can be dispatched as soon as
            // the buffer is available. Otherwise, both the AGU and the execution resource
            // must be available
            InstructionIssueDataDeps = getMemoryAddressIssueCycle (MemoryAddress);
#ifdef PRINT_DEPENDENCIES
             increaseInstructionValueInstance(&I);
#endif
            // New for memory model
            if (x86MemoryModel)
              InstructionIssueMemoryModel = LastLoadIssueCycle;
            InstructionIssueCycle =
            max (max
                 (max
                  (max
                   (max
                    (InstructionIssueFetchCycle,
                     InstructionIssueLoadBufferAvailable),
                    InstructionIssueLineFillBufferAvailable), InstructionIssueDataDeps), InstructionIssueCacheLineAvailable),
                 InstructionIssueMemoryModel);
            
            
            
            if ((ExtendedInstructionType != REGISTER_LOAD_NODE) && ConstraintAGUs) {
              // Once all previous constraints have been satisfied, check AGU availability, if any
#ifdef DEBUG_GENERIC
              DEBUG (dbgs () << "*********** Checking availability in AGUs *******************\n");
#endif
              //First, check in dedicated AGUs.
              if (nLoadAGUs > 0) {
                InstructionIssueLoadAGUAvailable =
                FindNextAvailableIssueCycle (InstructionIssueCycle, LOAD_ADDRESS_GENERATION_UNIT);
                
              }
              
              // Check in shared (loads/stores) AGUs if any, and if there is no available in
              // dedicated AGU
              if (!(nLoadAGUs > 0 && InstructionIssueLoadAGUAvailable == InstructionIssueCycle) && nAGUs > 0) {
                InstructionIssueAGUAvailable = FindNextAvailableIssueCycle (InstructionIssueCycle, ADDRESS_GENERATION_UNIT);
              }
              
              // Insert but check that there are AGUs.
              if (nLoadAGUs > 0 && InstructionIssueLoadAGUAvailable >= InstructionIssueAGUAvailable) {
                InsertNextAvailableIssueCycle (InstructionIssueLoadAGUAvailable, LOAD_ADDRESS_GENERATION_UNIT);
              }
              else {
                if (nAGUs > 0) {
                  InsertNextAvailableIssueCycle (InstructionIssueAGUAvailable, ADDRESS_GENERATION_UNIT);
                }
              }
              
              //Store specific AGU
              if (nLoadAGUs > 0) {
                InstructionIssueCycle =
                max (InstructionIssueCycle, min (InstructionIssueAGUAvailable, InstructionIssueLoadAGUAvailable));
              }
              else {
                InstructionIssueCycle = max (InstructionIssueCycle, InstructionIssueAGUAvailable);
              }
            }
            else {
              InstructionIssueAGUAvailable = InstructionIssueCycle;
            }
            
            
            Port = 0;

              if (ConstraintPorts) {
#ifdef DEBUG_GENERIC
                DEBUG (dbgs () << "*********** Checking availability in Ports and Resource *******************\n");
#endif
                //T here must be available cycle in both, the dispatch port
                // and the resource
                
                InstructionIssueThroughputAvailable =
                FindNextAvailableIssueCyclePortAndThroughtput (InstructionIssueCycle, ExtendedInstructionType,
                                                               NElementsVector);
              }
              else {
#ifdef DEBUG_GENERIC
                DEBUG (dbgs () << "*********** Checking availability in Resource *******************\n");
#endif
                
                InstructionIssueThroughputAvailable =
                FindNextAvailableIssueCycle (InstructionIssueCycle, ExecutionResource, NElementsVector);
                
                
                if (ConstraintPorts && DispatchPort[ExtendedInstructionType].size() > 0)
                  InsertNextAvailableIssueCycle (InstructionIssueThroughputAvailable, ExecutionResource, NElementsVector,
                                                 DispatchPort[ExtendedInstructionType][Port]);
                else
                  InsertNextAvailableIssueCycle (InstructionIssueThroughputAvailable, ExecutionResource, NElementsVector);
              }
            
            
            
            InstructionIssueCycle = max (InstructionIssueCycle, InstructionIssueThroughputAvailable);
            
            
            
#ifdef DEBUG_ISSUE_CYCLE
            DEBUG (dbgs () << "======== Instruction Issue Cycle (fetch cycle)" << InstructionIssueFetchCycle << "========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (LB availability)" << InstructionIssueLoadBufferAvailable <<
                   "========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (LFB availability)" << InstructionIssueLineFillBufferAvailable <<
                   "========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (cache line available)" << InstructionIssueCacheLineAvailable <<
                   "========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (data deps)" << InstructionIssueDataDeps << "========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (memory model Principle 1) " << InstructionIssueMemoryModel <<
                   "========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (AGU Availability)" << InstructionIssueAGUAvailable <<
                   "========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (Port Availability)" << InstructionIssuePortAvailable <<
                   "========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (Throughput Availability)" << InstructionIssueThroughputAvailable
                   << "========\n");
            // dbgs () << "__________________Instruction Issue Cycle " << InstructionIssueCycle << "__________________\n";
            DEBUG (dbgs () << "__________________Instruction Issue Cycle " << InstructionIssueCycle << "__________________\n");
#endif
          }
          
          break;
          // The Store can execute as soon as the value being stored is calculated
        case Instruction::Store:
          if (InstructionType >= 0 || forceAnalyze == true) {
            isLoad = false;
#ifdef INTERPRETER
            SmallString < 128 > StrVal;
            raw_svector_ostream OS (StrVal);
            OS << visitResult;
            MemoryAddress = strtol (OS.str ().str ().c_str (), NULL, 16);
#else
            MemoryAddress = addr;
#endif
            CacheLine = MemoryAddress >> BitsPerCacheLine;
            
            StoreCacheLine = CacheLine;
            LastAccess = getCacheLineLastAccess (StoreCacheLine);
            Info = getCacheLineInfo (StoreCacheLine);
            
            // Check whether the value is stored in a stack variable
            if (dyn_cast < AllocaInst > (I.getOperand(1))) {
              DEBUG (dbgs () << "Storing a value into a variable allocated in the stack (i.e., a \'register\')\n");
              // It is a store in the register file for sure, so although RegisterStackReuseDistance returns -1, it
              // has to be inserted in the register reuse stack and is a store to register. This is different
              // from a load that is not in register. If a load is not in the register file, it is inserted into
              // the register file, but RegisterStackDistance is -1 because it is a L1 load
              RegisterStackDistance = RegisterStackReuseDistance(MemoryAddress);
              RegisterStackDistance = 0;
            }
            if(RegisterStackDistance < 0){
              Distance = ReuseDistance (LastAccess, TotalInstructions, CacheLine);
            }
            ExtendedInstructionType = GetExtendedInstructionType (Instruction::Store, Distance,RegisterStackDistance );
            DEBUG (dbgs () << "Instruction Type " << GetNodeName(ExtendedInstructionType) << "\n");
#ifdef INT_FP_OPS
            if (ExtendedInstructionType == L1_STORE_NODE) {
              Q[0] += AccessGranularities[nArithmeticExecutionUnits + 1];
              Q[4] += AccessGranularities[nArithmeticExecutionUnits + 1];
              //fprintf(stderr, "\tL1\n");
            }
            else if (ExtendedInstructionType == L2_STORE_NODE) {
              Q[1] += AccessGranularities[nArithmeticExecutionUnits + 2];
              Q[4] += AccessGranularities[nArithmeticExecutionUnits + 2];
              //fprintf(stderr, "\tL2\n");
            }
            else if (ExtendedInstructionType == L3_STORE_NODE) {
              Q[2] += AccessGranularities[nArithmeticExecutionUnits + 3];
              Q[4] += AccessGranularities[nArithmeticExecutionUnits + 3];
              //fprintf(stderr, "\tL3\n");
            }
            else if (ExtendedInstructionType == MEM_STORE_NODE) {
              Q[3] += AccessGranularities[nArithmeticExecutionUnits + 4];
              Q[4] += AccessGranularities[nArithmeticExecutionUnits + 4];
              //fprintf(stderr, "\tMem\n");
            }
            else {
              report_fatal_error ("Store Mem op has nowhere to live");
            }
#endif
            
            ExecutionResource = ExecutionUnit[ExtendedInstructionType];
            Latency = ExecutionUnitsLatency[ExecutionResource];
            // Do not update anymore, udpate in InsertNextAvailableIssueCycle so that
            // ports are also updated. But need to update the InstructionsCount
            // UpdateInstructionCount(InstructionType, ExtendedInstructionType, NElementsVector, IsVectorInstruction);
            if(ExtendedInstructionType != REGISTER_STORE_NODE){
              if (IsVectorInstruction) {
                InstructionsCount[InstructionType] = InstructionsCount[InstructionType] + NElementsVector;
              }
              else
                InstructionsCount[InstructionType]++;
            }
            
#ifdef DEBUG_MEMORY_TRACES
            DEBUG (dbgs () << "MemoryAddress " << MemoryAddress << "\n");
            DEBUG (dbgs () << "CacheLine " << CacheLine << "\n");
#endif
#ifdef DEBUG_REUSE_DISTANCE
            DEBUG (dbgs () << "Store Last Access " << LastAccess << "\n");
            DEBUG (dbgs () << "Store latency " << Latency << "\n");
#endif
            
            InstructionIssueFetchCycle = InstructionFetchCycle;
            
            if (ExtendedInstructionType >= L1_STORE_NODE)
              InstructionIssueCacheLineAvailable = Info.IssueCycle;
            
            InstructionIssueDataDeps = getInstructionValueIssueCycle (&I);
#ifdef PRINT_DEPENDENCIES
            increaseInstructionValueInstance(&I);
#endif
            
#ifdef MOO_BUFFERS
            if(ExtendedInstructionType != REGISTER_STORE_NODE){
              //Calculate issue cycle depending on buffer Occupancy.
              if (StoreBufferSize > 0) {
                if (StoreBufferCompletionCycles.size () == StoreBufferSize) {	// If the store buffer is full
                  InstructionIssueStoreBufferAvailable = FindIssueCycleWhenStoreBufferIsFull ();
                  // If, moreover, the instruction has to go to the LineFillBuffer...
                  if (ExtendedInstructionType >= L2_LOAD_NODE && LineFillBufferSize > 0) {
                    if (LineFillBufferCompletionCycles.size () == (unsigned) LineFillBufferSize) {
                      InstructionIssueLineFillBufferAvailable = FindIssueCycleWhenLineFillBufferIsFull ();
                    }
                  }
                  
                }
                else {		// If the Store Buffer is not fulll...
                  if (ExtendedInstructionType >= L2_LOAD_NODE && LineFillBufferSize > 0) {	// If it has to go to the LFS...
                    
                    if (LineFillBufferCompletionCycles.size () == LineFillBufferSize || !DispatchToLineFillBufferQueue.empty ()) {
                      InstructionIssueLineFillBufferAvailable = FindIssueCycleWhenLineFillBufferIsFull ();
                    }
                    else {		// There is space on both
                      // Do nothing -> Instruction Issue Cycle is not affected by LB or LFB Occupancy
                      //Later, insert in both
                    }
                  }
                  else {		// It does not have to go to LFB...
                    //Do nothing (insert into LoadBuffer later, afte knowing IssueCycle
                    // depending on BW availability. Right not IssueCycle is not affected)
                  }
                }
              }
            }
#endif
            
            // New for memory model
            if ( ExtendedInstructionType != REGISTER_STORE_NODE && x86MemoryModel) {
              // Writes are not reordered with other writes
#ifdef DEBUG_GENERIC
              DEBUG (dbgs () << "LastStoreIssueCycle " << LastStoreIssueCycle << "\n");
              DEBUG (dbgs () << "LastLoadIssueCycle " << LastLoadIssueCycle << "\n");
#endif
              InstructionIssueMemoryModel = LastStoreIssueCycle;
              // Writes are not reordered with earlier reads
              // The memory-ordering model ensures that a store by a processor may not occur before a previous load by the same processor.
              InstructionIssueMemoryModel = max (InstructionIssueMemoryModel, LastLoadIssueCycle);
            }
                       InstructionIssueCycle =
            max (max
                 (max
                  (max (InstructionIssueFetchCycle, InstructionIssueStoreBufferAvailable), InstructionIssueDataDeps),
                  InstructionIssueCacheLineAvailable), InstructionIssueMemoryModel);
            
            if ((ExtendedInstructionType!= REGISTER_STORE_NODE) && ConstraintAGUs) {
              
              // Once all previous constraints have been satisfied, check AGU availability, if any
#ifdef DEBUG_GENERIC
              DEBUG (dbgs () << "*********** Checking availability in AGUs *******************\n");
#endif
              //First, check in dedicated AGUs.
              if (nStoreAGUs > 0) {
                InstructionIssueStoreAGUAvailable =
                FindNextAvailableIssueCycle (InstructionIssueCycle, STORE_ADDRESS_GENERATION_UNIT);
              }
              
              // Check in shared (loads/stores) AGUs if any, and if there is no available in
              // dedicated AGU
              if (!(nStoreAGUs > 0 && InstructionIssueStoreAGUAvailable == InstructionIssueCycle) && nAGUs > 0) {
                InstructionIssueAGUAvailable = FindNextAvailableIssueCycle (InstructionIssueCycle, ADDRESS_GENERATION_UNIT);
              }
              
              // Insert but check that there are AGUs.
              if (nStoreAGUs > 0 && InstructionIssueStoreAGUAvailable >= InstructionIssueAGUAvailable) {
                InsertNextAvailableIssueCycle (InstructionIssueStoreAGUAvailable, STORE_ADDRESS_GENERATION_UNIT);
              }
              else {
                if (nAGUs > 0) {
                  InsertNextAvailableIssueCycle (InstructionIssueAGUAvailable, ADDRESS_GENERATION_UNIT);
                }
              }
              
              //Store specific AGU
              
              if (nStoreAGUs > 0) {
                InstructionIssueCycle =
                max (InstructionIssueCycle, min (InstructionIssueAGUAvailable, InstructionIssueStoreAGUAvailable));
              }
              else {
                InstructionIssueCycle = max (InstructionIssueCycle, InstructionIssueAGUAvailable);
              }
              
            }
            else {
              InstructionIssueAGUAvailable = InstructionIssueCycle;
            }
            // When a cache line is writen does not impact when in can be loaded again.
            updateReuseDistanceDistribution (Distance, InstructionIssueCycle);
            //insertCacheLineInfo(StoreCacheLine, Info);
            insertCacheLineLastAccess (StoreCacheLine, TotalInstructions);
#ifdef DEBUG_GENERIC
            DEBUG (dbgs () << "CacheLine " << CacheLine << "\n");
            DEBUG (dbgs () << "StoreCacheLine " << StoreCacheLine << "\n");
            
#endif
            
            // If there is a store buffer, the dipatch cycle might be different from
            // the issue (execution) cycle.
            if (0) {
              if (ConstraintPorts) {
                bool FirstPortAvailableFound = false;
                DEBUG (dbgs () << "*********** Checking availability in Ports *******************\n");
                
                // Find next available issue cycle in the port as soon as the store
                // buffer is available (and all previous constraints have been satisfied)
                InstructionIssuePortAvailable = InstructionIssueCycle;
#ifdef DEBUG_GENERIC
                DEBUG (dbgs () << "ExtendedInstructionType " << GetNodeName (ExtendedInstructionType) << "\n");
                DEBUG (dbgs () << "ExecutionResource " << GetResourceName (ExecutionResource) << "\n");
#endif
                for (unsigned i = 0; i < DispatchPort[ExtendedInstructionType].size (); i++) {
#ifdef DEBUG_GENERIC
                  DEBUG (dbgs () << "Finding availability in port " << GetResourceName (DispatchPort[ExtendedInstructionType]
                                                                                        [i]) << "\n");
#endif
                  InstructionIssuePortAvailable =
                  FindNextAvailableIssueCycle (InstructionIssueCycle, DispatchPort[ExtendedInstructionType][i]);
                  if (InstructionIssuePortAvailable != InstructionIssueCycle) {
                    if (FirstPortAvailableFound == false){
                      //if (i == 0)
                      FirstPortAvailableFound = true;
                      InstructionIssueCycleFirstTimeAvailable = InstructionIssuePortAvailable;
                    }
                    else {
                      InstructionIssuePortAvailable =
                      min (InstructionIssuePortAvailable, InstructionIssueCycleFirstTimeAvailable);
                      if (InstructionIssuePortAvailable == InstructionIssueCycleFirstTimeAvailable) {
                        Port = i;
                        if (InstructionIssuePortAvailable == InstructionIssueCycle)
                          break;
                      }
                    }
                  }
                  else {
                    // If Node is NULL, it is available for sure.
#ifdef DEBUG_GENERIC
                    DEBUG (dbgs () << "Port is " << GetResourceName (DispatchPort[ExtendedInstructionType]
                                                                     [i]) << "\n");
#endif
                    Port = i;
                    break;
                  }
                }
                
                
                InsertNextAvailableIssueCycle (InstructionIssuePortAvailable, DispatchPort[ExtendedInstructionType][Port]);
                
                InstructionIssueCycle = max (InstructionIssueCycle, InstructionIssuePortAvailable);
              }
              else {
                InstructionIssuePortAvailable = InstructionIssueCycle;
              }
              // Finally, calculate issue cycle depending on resource availability
#ifdef DEBUG_GENERIC
              DEBUG (dbgs () << "*********** Checking availability in Resource *******************\n");
              DEBUG (dbgs () << "Checking availability in cycle " << InstructionIssueCycle << "\n");
              
#endif
              InstructionIssueThroughputAvailable =
              FindNextAvailableIssueCycle (InstructionIssueCycle, ExecutionResource, NElementsVector);
              
              InsertNextAvailableIssueCycle (InstructionIssueThroughputAvailable, ExecutionResource, NElementsVector,
                                             DispatchPort[ExtendedInstructionType][Port]);
              
              
            }
            else {
              if (ConstraintPorts) {
                
                InstructionIssueThroughputAvailable =
                FindNextAvailableIssueCyclePortAndThroughtput (InstructionIssueCycle, ExtendedInstructionType,
                                                               NElementsVector);
              }
              else {
                
                InstructionIssueThroughputAvailable =
                FindNextAvailableIssueCycle (InstructionIssueCycle, ExecutionResource, NElementsVector);
                
                if (DispatchPort[ExtendedInstructionType].size() > 0)
                  InsertNextAvailableIssueCycle (InstructionIssueThroughputAvailable, ExecutionResource, NElementsVector,
                                                 DispatchPort[ExtendedInstructionType][Port]);
                else
                  InsertNextAvailableIssueCycle (InstructionIssueThroughputAvailable, ExecutionResource, NElementsVector);
                
              }
            }
            
            
            
            InstructionIssueCycle = max (InstructionIssueCycle, InstructionIssueThroughputAvailable);
#ifdef DEBUG_ISSUE_CYCLE
            DEBUG (dbgs () << "======== Instruction Issue Cycle (fetch cycle) " << InstructionIssueFetchCycle << " ========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (SB availability) " << InstructionIssueStoreBufferAvailable <<
                   " ========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (cache line available) " << InstructionIssueCacheLineAvailable <<
                   " ========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (data deps)" << InstructionIssueDataDeps << " ========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (memory model Principles 2 and 3) " <<
                   InstructionIssueMemoryModel << " ========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (AGU Availability) " << InstructionIssueAGUAvailable <<
                   "========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (Port Availability) " << InstructionIssuePortAvailable <<
                   "========\n");
            DEBUG (dbgs () << "======== Instruction Issue Cycle (Throughput Availability) " <<
                   InstructionIssueThroughputAvailable << "========\n");
            DEBUG (dbgs () << "__________________Instruction Issue Cycle " << InstructionIssueCycle << "__________________\n");
            // dbgs () << "__________________Instruction Issue Cycle " << InstructionIssueCycle << "__________________\n";
#endif
          }
          break;
          
        case Instruction::Ret:
          // Determine the uses of the returned value outside the funcion
          //(i.e., the uses of the calling function)
          // Check http://llvm.org/docs/ProgrammersManual.html for a lot
          // of info about how iterate through functions, bbs, etc.
          F = I.getParent ()->getParent ();
          InstructionIssueCycle = max (max (InstructionFetchCycle, BasicBlockBarrier), getInstructionValueIssueCycle (&I));
          //#ifdef INTERPRETER
#if LLVM_VERSION_MINOR<6
          for (Value::use_iterator IT = F->use_begin (), ET = F->use_end (); IT != ET; ++IT) {
            // Iterate over the users of the uses of the function
            for (Value::use_iterator it = (*IT)->use_begin (), ite = (*IT)->use_end (); it != ite; ++it) {
              insertInstructionValueIssueCycle (*it, InstructionIssueCycle);
            }
          }
#else
          
          for (User * U:F->users ()) {
            //  if (Instruction *Inst = dyn_cast<Instruction>(U)) {
            for (User * UI:U->users ()) {
              // if (Instruction *i = dyn_cast<Instruction>(UI)) {
              insertInstructionValueIssueCycle (UI, InstructionIssueCycle);
              
              // }
            }
            //}
          }
#endif
          break;
          
          //-------------------------General case------------------------------//
        default:
          if (InstructionType == 0 || forceAnalyze == true) {
            OriginalInstructionIssueCycle = getInstructionValueIssueCycle (&I);
#ifdef PRINT_DEPENDENCIES
		  increaseInstructionValueInstance(&I);
#endif
            InstructionIssueCycle = max (max (InstructionFetchCycle, BasicBlockBarrier), OriginalInstructionIssueCycle);
            
            ExtendedInstructionType = GetExtendedInstructionType (OpCode);
            // UpdateInstructionCount(InstructionType, ExtendedInstructionType,  NElementsVector, IsVectorInstruction);
            if (IsVectorInstruction) {
              InstructionsCount[InstructionType] = InstructionsCount[InstructionType] + NElementsVector;
            }
            else
              InstructionsCount[InstructionType]++;
            
            Latency = ExecutionUnitsLatency[ExtendedInstructionType];
            
            if (InstructionIssueCycle > OriginalInstructionIssueCycle) {
              NInstructionsStalled[ExtendedInstructionType]++;
            }
            
            
            
#ifdef DEBUG_GENERIC
            DEBUG (dbgs () << "*********** Checking availability in Resource *******************\n");
#endif
            InstructionIssueThroughputAvailable =
            FindNextAvailableIssueCyclePortAndThroughtput (InstructionIssueCycle, ExtendedInstructionType, NElementsVector);
            
            InstructionIssueCycle = max (InstructionIssueCycle, InstructionIssueThroughputAvailable);
#ifdef DEBUG_ISSUE_CYCLE
            DEBUG (dbgs () << "========Original Instruction Issue Cycle (data deps)" << OriginalInstructionIssueCycle <<
                   "========\n");
            DEBUG (dbgs () << "========Original Instruction Issue Cycle (fetch cycle) " << InstructionFetchCycle <<
                   "========\n");
            DEBUG (dbgs () << "__________________Instruction Issue Cycle " << InstructionIssueCycle << "__________________\n");
#endif
          }
          break;
      }
      
      
      
      
      if (InstructionType >= 0 || forceAnalyze == true) {
#ifdef SOURCE_CODE_ANALYSIS
        SourceCodeLineOperations[SourceCodeLine].insert (ExecutionUnit[ExtendedInstructionType]);
#endif
        uint64_t NewInstructionIssueCycle = InstructionIssueCycle;
        
        
        if (x86MemoryModel) {
          if (OpCode == Instruction::Load && ExtendedInstructionType >= L1_LOAD_NODE) {
            LastLoadIssueCycle = NewInstructionIssueCycle;
          }
          else {
            if (OpCode == Instruction::Store && ExtendedInstructionType >= L1_STORE_NODE)
              LastStoreIssueCycle = NewInstructionIssueCycle;
          }
        }
        
        // A load can execute as soon as all its operands are available, i.e., all
        // the values that are being loaded. If the same value is loaded again, without
        // having been used in between (Read After Read dependence), then the next load
        // can only be issued after the first one has finished.
        // This only applies to memory accesses > L1. If we access a cache line at cycle
        // X which is in L3, e.g., it has a latency of 30. The next  time this cache line
        // is accessed it is in L1, but it is inconsistent to assume that it can be
        // loaded also at cycle X and have a latency of 4 cycles.
        
        ExecutionResource = ExecutionUnit[ExtendedInstructionType];
        
        if (OpCode == Instruction::Load && RARDependences) {
          
          if(RegisterFileSize != 0){
            
            
            insertMemoryAddressIssueCycle (MemoryAddress, NewInstructionIssueCycle + Latency);
            
          }else{
            if(ExtendedInstructionType > L1_LOAD_NODE
               && ExecutionUnitsLatency[ExecutionResource] > ExecutionUnitsLatency[L1_LOAD_CHANNEL]){
              Info = getCacheLineInfo (CacheLine);
              Info.IssueCycle = NewInstructionIssueCycle + Latency;
              
              insertCacheLineInfo (CacheLine, Info);
              insertMemoryAddressIssueCycle (MemoryAddress, NewInstructionIssueCycle + Latency);
            }
            
          }
          
        }
        

        if (OpCode == Instruction::Store) {
          if(RegisterFileSize!=0){
#ifdef DEBUG_GENERIC
            DEBUG (dbgs () << "Inserting issue cycle " << NewInstructionIssueCycle +
                   Latency << " for cache line " << StoreCacheLine << "\n");
#endif
            //Info = getCacheLineInfo (StoreCacheLine);
            // Info.IssueCycle = NewInstructionIssueCycle + Latency;
            //  insertCacheLineInfo (StoreCacheLine, Info);
            //  else
            insertMemoryAddressIssueCycle (MemoryAddress, NewInstructionIssueCycle + Latency);
          }else{
            if( ExtendedInstructionType > L1_STORE_NODE
               && ExecutionUnitsLatency[ExecutionResource] > ExecutionUnitsLatency[L1_LOAD_CHANNEL]){
              Info = getCacheLineInfo (StoreCacheLine);
              Info.IssueCycle = NewInstructionIssueCycle + Latency;
              insertCacheLineInfo (StoreCacheLine, Info);
              //  else
              insertMemoryAddressIssueCycle (MemoryAddress, NewInstructionIssueCycle + Latency);
            }
          }
        }
        
        // =========================== SPATIAL PREFETCHER ======================================
        
        if (SpatialPrefetcher && (OpCode == Instruction::Load || OpCode == Instruction::Store)
            && ExtendedInstructionType > PrefetchDispatch && !(ExecutionUnit[ExtendedInstructionType] == PrefetchLevel)
            
            ) {
          NextCacheLine = CacheLine + 1;
          
          //Get reuse distance of NextCacheLine
          Info = getCacheLineInfo (NextCacheLine);
          Distance = ReuseDistance (Info.LastAccess, TotalInstructions, NextCacheLine, true);
          NextCacheLineExtendedInstructionType = GetMemoryInstructionType (Distance, MemoryAddress, isLoad);
          
          
          ExecutionResource = ExecutionUnit[NextCacheLineExtendedInstructionType];
          LatencyPrefetch = ExecutionUnitsLatency[ExecutionResource] - ExecutionUnitsLatency[PrefetchDestination];
          
          
#ifdef DEBUG_PREFETCHER
          DEBUG (dbgs () << "CacheLine " << CacheLine << "\n");
          DEBUG (dbgs () << "NextCacheLine " << NextCacheLine << "\n");
          DEBUG (dbgs () << "Execution Resource (for bandwidth consumption) " << GetResourceName (ExecutionResource) << "\n");
          DEBUG (dbgs () << "Latency (for bandwidth consumption) " << LatencyPrefetch << "\n");
          
#endif
          
          // Prefetch every time there is a miss (not necesarly an access to memory), but only if the prefetched
          //data is in memoty.
          if (ExecutionResource > PrefetchTarget && ExecutionResource >= PrefetchDestination) {
            InstructionsCountExtended[NextCacheLineExtendedInstructionType]++;
            if (IsVectorInstruction) {
              InstructionsCount[InstructionType] = InstructionsCount[InstructionType] + NElementsVector;
            }
            else
              InstructionsCount[InstructionType]++;
            // UpdateReuseDistribution
            NextCacheLineIssueCycle = FindNextAvailableIssueCycle (NewInstructionIssueCycle, ExecutionResource);
            
            updateReuseDistanceDistribution (Distance, NextCacheLineIssueCycle);
            
#ifdef DEBUG_PREFETCHER
            DEBUG (dbgs () << "Prefetching next cache line at cycle " << NextCacheLineIssueCycle << "\n");
#endif
            
            InsertNextAvailableIssueCycle (NextCacheLineIssueCycle, ExecutionResource, 1, 0, true);
            
            
            
            Info.IssueCycle = NextCacheLineIssueCycle + LatencyPrefetch;
            Info.LastAccess = TotalInstructions;
            insertCacheLineInfo (NextCacheLine, Info);
#ifdef DEBUG_GENERIC
            DEBUG (dbgs () << "Inserting issue cycle " << NextCacheLineIssueCycle +
                   LatencyPrefetch << " for cache line " << NextCacheLine << "\n");
#endif
          }
        }
        
        //======================= END NEW CODE ==========================================//
        
        //Iterate over the uses of the generated value (except for GetElementPtr)
        if (OpCode != Instruction::GetElementPtr) {

#if LLVM_VERSION_MINOR<6
#ifdef PRINT_DEPENDENCIES
			dbgs()<<  &I << " " << getInstructionValueInstance(&I) << " ";
#endif 		
          for (Value::use_iterator i = I.use_begin (), ie = I.use_end (); i != ie; ++i) {
#ifdef DEBUG_DEPS_FUNCTION_CALL
            DEBUG (dbgs () << "Setting use  " << *i << " to " << NewInstructionIssueCycle + Latency << "\n");
#endif
#ifdef PRINT_DEPENDENCIES
            dbgs () <<  *i <<" " << getInstructionValueInstance(*i) << " ";
#endif
            if (dyn_cast < PHINode > (*i)) {
              insertInstructionValueIssueCycle (*i, NewInstructionIssueCycle + Latency, true);
            }
            else {
              insertInstructionValueIssueCycle (*i, NewInstructionIssueCycle + Latency);
            }
            if (dyn_cast < CallInst > (*i)) {
#ifdef DEBUG_DEPS_FUNCTION_CALL
              DEBUG (dbgs () << "The use is a call to function\n");
#endif
              CS = CallSite (*i);
              F = CS.getCalledFunction ();
              // Loop over the arguments of the called function --- From Execution.cpp
              NumArgs = CS.arg_size ();
              ArgVals.reserve (NumArgs);
              for (CallSite::arg_iterator j = CS.arg_begin (), e = CS.arg_end (); j != e; ++j) {
                Value *V = *j;
                ArgVals.push_back (V);
              }
              
              // Make sure it is an LLVM-well-defined funciton
              if (static_cast < Function * >(F)) {
                for (Function::arg_iterator AI = F->arg_begin (), E = F->arg_end (); AI != E; ++AI, ++k) {
#ifdef DEBUG_DEPS_FUNCTION_CALL
                  DEBUG (dbgs () << "Iterate through the arguments of the call\n");
#endif
                  /*   Latency = (&AI->getOpcode() == Instruction::Load || (ArgVals[j])->getOpcode()
                   == Instruction::Store)? MEM_LATENCY:FLOP_LATENCY; */
#ifdef DEBUG_DEPS_FUNCTION_CALL
                  DEBUG (dbgs () << "Argument " << ArgVals[k] << "\n");
#endif
                  /* //Iterate through the uses of the argument?
                   insertInstructionValueIssueCycle(AI, InstructionIssueCycle+1); // +1???
                   */
#ifdef DEBUG_DEPS_FUNCTION_CALL
                  DEBUG (dbgs () << "Iterate through the uses of the argument\n");
#endif
                  if (ArgVals[k] == &I) {
#ifdef DEBUG_DEPS_FUNCTION_CALL
                    DEBUG (dbgs () << "Argument equal to current instruction\n");
#endif
                    for (Value::use_iterator vi = (*AI).use_begin (), vie = (*AI).use_end (); vi != vie; ++vi) {
#ifdef DEBUG_DEPS_FUNCTION_CALL
                      DEBUG (dbgs () << "Use of the argument " << *vi << "\n");
#endif
                      insertInstructionValueIssueCycle (*vi, NewInstructionIssueCycle + Latency);
                    }
                  }
                }
              }
            }
          }
            dbgs () <<"\n";	
#else
          // No interpreter: new way of iterating through the uses of an instruction.
          // TODO: Fix also iterating through the arguments
		dbgs()<< "Current instruction " << *I << " ("<< I <<")\n";
          for (User * U:I.users ()) {
            //  if (Instruction *i = dyn_cast<Instruction>(U)) {
            // for(Value::use_iterator i = I.use_begin(), ie = I.use_end(); i!=ie; ++i){
            
#ifdef DEBUG_DEPS_FUNCTION_CALL
            DEBUG (dbgs () << "Setting use  " << U << " to " << NewInstructionIssueCycle + Latency << "\n");
#endif
		dbgs () << "Setting use  " << U << " to " << NewInstructionIssueCycle + Latency << "\n";
            if (dyn_cast < PHINode > (U)) {
              insertInstructionValueIssueCycle (U, NewInstructionIssueCycle + Latency, true);
            }
            else {
              insertInstructionValueIssueCycle (U, NewInstructionIssueCycle + Latency);
            }
            if (dyn_cast < CallInst > (U)) {
#ifdef DEBUG_DEPS_FUNCTION_CALL
              DEBUG (dbgs () << "The use is a call to function\n");
#endif
              CS = CallSite (U);
              F = CS.getCalledFunction ();
              // Loop over the arguments of the called function --- From Execution.cpp
              NumArgs = CS.arg_size ();
              ArgVals.reserve (NumArgs);
              for (CallSite::arg_iterator j = CS.arg_begin (), e = CS.arg_end (); j != e; ++j) {
                Value *V = *j;
                ArgVals.push_back (V);
              }
              
              // Make sure it is an LLVM-well-defined funciton
              if (static_cast < Function * >(F)) {
                for (Function::arg_iterator AI = F->arg_begin (), E = F->arg_end (); AI != E; ++AI, ++k) {
#ifdef DEBUG_DEPS_FUNCTION_CALL
                  DEBUG (dbgs () << "Iterate through the arguments of the call\n");
#endif
                  /*   Latency = (&AI->getOpcode() == Instruction::Load || (ArgVals[j])->getOpcode()
                   == Instruction::Store)? MEM_LATENCY:FLOP_LATENCY; */
#ifdef DEBUG_DEPS_FUNCTION_CALL
                  DEBUG (dbgs () << "Argument " << ArgVals[k] << "\n");
#endif
                  /* //Iterate through the uses of the argument?
                   insertInstructionValueIssueCycle(AI, InstructionIssueCycle+1); // +1???
                   */
#ifdef DEBUG_DEPS_FUNCTION_CALL
                  DEBUG (dbgs () << "Iterate through the uses of the argument\n");
#endif
                  if (ArgVals[k] == &I) {
#ifdef DEBUG_DEPS_FUNCTION_CALL
                    DEBUG (dbgs () << "Argument equal to current instruction\n");
#endif
                    for (Value::use_iterator vi = (*AI).use_begin (), vie = (*AI).use_end (); vi != vie; ++vi) {
#ifdef DEBUG_DEPS_FUNCTION_CALL
                      DEBUG (dbgs () << "Use of the argument " << *vi << "\n");
#endif
                      insertInstructionValueIssueCycle (*vi, NewInstructionIssueCycle + Latency);
                    }
                  }
                }
              }
            }
          }
#endif
        }
        
        
        
        if (forceAnalyze == true) {
          if (OpCode == Instruction::Load) {
#ifdef DEBUG_GENERIC
            DEBUG (dbgs () << "Forzing uses of instruction " << &I << "to " << NewInstructionIssueCycle + Latency << "\n");
#endif
            insertInstructionValueIssueCycle (&I, NewInstructionIssueCycle + Latency);
          }
          else {
#ifdef DEBUG_GENERIC
            DEBUG (dbgs () << "Forzing uses of instruction " << &I << "to " << NewInstructionIssueCycle + Latency << "\n");
#endif
            insertInstructionValueIssueCycle (&I, NewInstructionIssueCycle);
          }
        }
        
        //========================= Update Parallelism Distribution ===================//
        
#ifdef ILP_DISTRIBUTION
        if (InstructionType >= 0) {	// Consider only selected instructions types
          if (ParallelismDistribution.empty ()) {
            for (uint i = 0; i < Latency; i++) {
              vector < unsigned >v;
              ParallelismDistribution.push_back (v);
              for (j = 0; j < N_INST_TYPES; j++)
                ParallelismDistribution[InstructionIssueCycle + i].push_back (0);
            }
          }
          else {
            uint DistributionSize = ParallelismDistribution.size ();
            if (DistributionSize <= InstructionIssueCycle + Latency - 1) {	// Is not empty, but distribution has to be extended
              uint extraSlots = max (InstructionIssueCycle + Latency - DistributionSize, (unsigned long long) 1);
              for (uint i = 0; i < extraSlots; i++) {
                vector < unsigned >v;
                ParallelismDistribution.push_back (v);
                for (j = 0; j < N_INST_TYPES; j++)
                  ParallelismDistribution[DistributionSize + i].push_back (0);
              }
            }
          }
          for (uint i = 0; i < Latency; i++)
            ParallelismDistribution[InstructionIssueCycle + i][InstructionType]++;
        }
#endif
        
        //---------------------- End of Update Parallelism Distribution--------------------//
        
        // ------------ Work with limited instruction issue window ----------------//
        
        //When InstructionFetchBandwidth is INF, remaining instructions to fetch
        // is -1, but still load and stores must be inserted into the OOO buffers
        if(ExtendedInstructionType != REGISTER_LOAD_NODE && ExtendedInstructionType != REGISTER_STORE_NODE  && isRegisterSpill==false){
          if (RemainingInstructionsFetch > 0 || RemainingInstructionsFetch == INF) {
            
            if (RemainingInstructionsFetch > 0){
              RemainingInstructionsFetch--;
              
            }
            
            uint64_t CycleInsertReservationStation = 0;
            if (OpCode == Instruction::Load) {
              // If LB is not full, they go directly to the LB and to the RS
              // If LB is INF, this comparison is false. But still
              // we need to check wether RS is INF
              
              bool BufferFull=false;
              if(SmallBuffers){
                if (LoadBufferCompletionCycles.size() == LoadBufferSize)
                  BufferFull = true;
              }else{
                if (node_size (LoadBufferCompletionCyclesTree) == LoadBufferSize)
                  BufferFull = true;
              }
              if (LoadBufferSize > 0 && BufferFull) {
                //if (node_size (LoadBufferCompletionCyclesTree) == LoadBufferSize && LoadBufferSize > 0) {
                // if(LoadBufferCompletionCycles.size() == LoadBufferSize && LoadBufferSize > 0){
                
                // Put in the reservation station, but only if RS exists
                if(SmallBuffers)
                  CycleInsertReservationStation = FindIssueCycleWhenLoadBufferIsFull();
                else
                  CycleInsertReservationStation = FindIssueCycleWhenLoadBufferTreeIsFull ();
                
                if(InstructionIssueLoadBufferAvailable != CycleInsertReservationStation)
                  report_fatal_error("InstructionIssueLoadBufferAvailable != CycleInsertReservationStation");
                //  CycleInsertReservationStation =  InstructionIssueLoadBufferAvailable;
                ReservationStationIssueCycles.push_back (CycleInsertReservationStation);
                
                //Put in the DispatchToLoadBufferQueue
                if(SmallBuffers){
                  
                  InstructionDispatchInfo DispathInfo;
                  DispathInfo.IssueCycle = CycleInsertReservationStation;
                  DispathInfo.CompletionCycle = NewInstructionIssueCycle+Latency;
                  DispatchToLoadBufferQueue.push_back(DispathInfo);
                  
                }else{
                  
                  
                  if (DispatchToLoadBufferQueueTree == NULL) {
                    MaxDispatchToLoadBufferQueueTree = CycleInsertReservationStation;
                  }
                  else {
                    MaxDispatchToLoadBufferQueueTree = max (MaxDispatchToLoadBufferQueueTree, CycleInsertReservationStation);
                    
                  }
                  
#ifdef DEBUG_OOO_BUFFERS
                  DEBUG (dbgs () << "MaxDispatchToLoadBufferQueueTree  " << MaxDispatchToLoadBufferQueueTree <<"\n");
                  DEBUG (dbgs () << "CycleInsertReservationStation  " << CycleInsertReservationStation <<"\n");
                  DEBUG (dbgs () << "Inserting  " << NewInstructionIssueCycle + Latency << " to DispatchToLoadBufferQueue, whith dispatch cycle "<<MaxDispatchToLoadBufferQueueTree <<"\n");
               
                  
#endif

                  
                  DispatchToLoadBufferQueueTree =
                  insert_node (NewInstructionIssueCycle + Latency, MaxDispatchToLoadBufferQueueTree, DispatchToLoadBufferQueueTree);
                  //	PrintLoadBufferTree();
                  //PrintDispatchToLoadBufferTree();
                  
                }
                

                
                
#ifdef SOURCE_CODE_ANALYSIS
                SourceCodeLineOperations[SourceCodeLine].insert (LB_STALL);
                
                unsigned TreeChunk = 0;
                for (uint64_t i = InstructionFetchCycle; i < CycleInsertReservationStation; i++) {
                  TreeChunk = GetTreeChunk (i);
                  // SUPER FullOccupancyCyclesTree[TreeChunk].insert_node(i, LB_STALL);
                  
                  
                  
#ifdef DEBUG_SOURCE_CODE_ANALYSIS
                  DEBUG (dbgs () << "LB_STALL for source code line " << SourceCodeLine << " in cycle " << i << "\n");
#endif
                  
                  // TODO: Why is this whithin a source code analysis? FullOccupancyCyclesTree[TreeChunk] = insert_node(i, LB_STALL,FullOccupancyCyclesTree[TreeChunk]);
                  
                  FullOccupancyCyclesTree[TreeChunk].insert_source_code_line (i, SourceCodeLine, LB_STALL);
                  //   FullOccupancyCyclesTree[TreeChunk]->SourceCodeLines.insert(SourceCodeLine);
                  
                }
#endif
                
                
                
                // If, moreover, the instruction has to go to the LineFillBuffer...
                if (ExtendedInstructionType >= L2_LOAD_NODE && LineFillBufferSize > 0) {
                  if (LineFillBufferCompletionCycles.size () == LineFillBufferSize || !DispatchToLineFillBufferQueue.empty ()) {
                    InstructionDispatchInfo DispathInfo;
                    DispathInfo.IssueCycle = FindIssueCycleWhenLineFillBufferIsFull ();
                    DispathInfo.CompletionCycle = NewInstructionIssueCycle + Latency;
                    DispatchToLineFillBufferQueue.push_back (DispathInfo);
#ifdef DEBUG_OOO_BUFFERS
                    DEBUG (dbgs () << "Inserting  " << DispathInfo.IssueCycle << " to DispatchToLineFillBufferQueue\n");
#endif
                    
                    
#ifdef SOURCE_CODE_ANALYSIS
                    SourceCodeLineOperations[SourceCodeLine].insert (LFB_STALL);
                    
                    
                    TreeChunk = 0;
                    for (uint64_t i = InstructionFetchCycle; i < CycleInsertReservationStation; i++) {
                      TreeChunk = GetTreeChunk (i);
                      
                      SourceCodeLineOperations[SourceCodeLine].insert (LFB_STALL);
#ifdef DEBUG_SOURCE_CODE_ANALYSIS
                      DEBUG (dbgs () << "LFB_STALL for source code line " << SourceCodeLine << " in cycle " << i << "\n");
#endif
                      
                      
                      
                      FullOccupancyCyclesTree[TreeChunk].insert_source_code_line (i, SourceCodeLine, LFB_STALL);
                      
                    }
                    
#endif
                    
                  }
                  else {		// There is space on both
#ifdef DEBUG_OOO_BUFFERS
                    DEBUG (dbgs () << "Inserting  " << NewInstructionIssueCycle + Latency << " to Line Fill Buffer\n");
#endif
                    LineFillBufferCompletionCycles.push_back (NewInstructionIssueCycle + Latency);
                  }
                }
              }
              else {
                bool BufferFull=true;
                if(SmallBuffers){
                  if (LoadBufferCompletionCycles.size() != LoadBufferSize)
                    BufferFull = false;
                }else{
                  if (node_size (LoadBufferCompletionCyclesTree) != LoadBufferSize)
                    BufferFull = false;
                }
                
                
                //If LB is not full
                if (LoadBufferSize > 0 && BufferFull==false) {

                  //Insert into LB
                  if(SmallBuffers)
                    LoadBufferCompletionCycles.push_back(NewInstructionIssueCycle+Latency);
                  else{
#ifdef DEBUG_OOO_BUFFERS
                    DEBUG (dbgs () << "node_size (LoadBufferCompletionCyclesTree)  " << node_size (LoadBufferCompletionCyclesTree) << "\n");
                    DEBUG (dbgs () << "Inserting  " << NewInstructionIssueCycle + Latency << " to LoadBuffer\n");
#endif
                    if (node_size (LoadBufferCompletionCyclesTree) == 0) {
                      MinLoadBuffer = NewInstructionIssueCycle + Latency;
                    }
                    else {
                      MinLoadBuffer = min (MinLoadBuffer, NewInstructionIssueCycle + Latency);
                      
                    }
                    LoadBufferCompletionCyclesTree =
                    insert_node (NewInstructionIssueCycle + Latency, LoadBufferCompletionCyclesTree);
                  }
                  if (ExtendedInstructionType >= L2_LOAD_NODE && LineFillBufferSize != 0) {	// If it has to go to the LFS...
                    
                    if (LineFillBufferCompletionCycles.size () == LineFillBufferSize || !DispatchToLineFillBufferQueue.empty ()) {
                      InstructionDispatchInfo DispathInfo;
                      DispathInfo.IssueCycle = FindIssueCycleWhenLineFillBufferIsFull ();
                      DispathInfo.CompletionCycle = NewInstructionIssueCycle + Latency;
                      DispatchToLineFillBufferQueue.push_back (DispathInfo);
#ifdef DEBUG_OOO_BUFFERS
                      DEBUG (dbgs () << "Inserting  " << DispathInfo.IssueCycle << " to DispatchToLineFillBufferQueue\n");
#endif
                    }
                    else {		// There is space on both
#ifdef DEBUG_OOO_BUFFERS
                      DEBUG (dbgs () << "Inserting  " << NewInstructionIssueCycle + Latency << " to Line Fill Buffer\n");
#endif
                      LineFillBufferCompletionCycles.push_back (NewInstructionIssueCycle + Latency);
                    }
                  }
                  else {		// It does not have to go to LFB...
                    // Insert into LoadBuffer, what we have already done.
                  }
                }
                else {
                  //If LB is zero.... Insert into into RS, if it exists
                  if (ReservationStationSize > 0) {
                    CycleInsertReservationStation = NewInstructionIssueCycle;
                    ReservationStationIssueCycles.push_back (CycleInsertReservationStation);
                  }
                }
              }
            }
            else {
              if (OpCode == Instruction::Store) {
                if (StoreBufferCompletionCycles.size () == StoreBufferSize && StoreBufferSize > 0) {
                  
                  CycleInsertReservationStation = FindIssueCycleWhenStoreBufferIsFull ();
                  ReservationStationIssueCycles.push_back (CycleInsertReservationStation);
                  InstructionDispatchInfo DispathInfo;
                  //  DispathInfo.IssueCycle = NewInstructionIssueCycle;
                  //TODO: Isn't it CycleInsertReservationStation?
                  DispathInfo.IssueCycle = FindIssueCycleWhenStoreBufferIsFull ();
                  DispathInfo.CompletionCycle = NewInstructionIssueCycle + Latency;
                  DispatchToStoreBufferQueue.push_back (DispathInfo);
#ifdef DEBUG_OOO_BUFFERS
                  DEBUG (dbgs () << "Inserting  " << NewInstructionIssueCycle << " to DispatchToStoreBufferQueue\n");
#endif
#ifdef SOURCE_CODE_ANALYSIS
                  SourceCodeLineOperations[SourceCodeLine].insert (SB_STALL);
                  
                  TreeChunk = 0;
                  for (uint64_t i = InstructionFetchCycle; i < CycleInsertReservationStation; i++) {
                    TreeChunk = GetTreeChunk (i);
                    
#ifdef DEBUG_SOURCE_CODE_ANALYSIS
                    DEBUG (dbgs () << "SB_STALL for source code line " << SourceCodeLine << " in cycle " << i << "\n");
#endif
                    
                    
                    
                    FullOccupancyCyclesTree[TreeChunk].insert_source_code_line (i, SourceCodeLine, SB_STALL);
                    
                  }
#endif
                  
                }
                else {		// If it is not full
                  if (StoreBufferCompletionCycles.size () != StoreBufferSize && StoreBufferSize > 0) {
#ifdef DEBUG_OOO_BUFFERS
                    DEBUG (dbgs () << "Inserting  " << NewInstructionIssueCycle + Latency << " to StoreBuffer\n");
#endif
                    StoreBufferCompletionCycles.push_back (NewInstructionIssueCycle + Latency);
                  }
                  else {
                    // If StoreBufferSize == 0, insert into RS if it exists
                    if (ReservationStationSize > 0) {
                      CycleInsertReservationStation = NewInstructionIssueCycle;
                      ReservationStationIssueCycles.push_back (CycleInsertReservationStation);
                    }
                  }
                }
              }
              else {
                // Not load nor store -> Insert into RS if its size is > -1
                if (ReservationStationSize > 0) {
                  CycleInsertReservationStation = NewInstructionIssueCycle;
                  ReservationStationIssueCycles.push_back (CycleInsertReservationStation);
                }
              }
            }
            
            
            if (InOrderExecution) {
              ReorderBufferCompletionCycles.push_back (NewInstructionIssueCycle);
            }
            else {
              if (ReorderBufferSize > 0) {
                ReorderBufferCompletionCycles.push_back (NewInstructionIssueCycle + Latency);
              }
            }
            
#ifdef DEBUG_OOO_BUFFERS
            if (ReservationStationSize > 0 && CycleInsertReservationStation > 0)
              DEBUG (dbgs () << "Inserting  " << CycleInsertReservationStation << " to ReservationStationIssueCycles\n");
            if (ReorderBufferSize > 0)
              DEBUG (dbgs () << "Inserting  " << NewInstructionIssueCycle + Latency << " to ReorderBufferCompletionCycles\n");
#endif
          }
        }
      }
    }
#ifdef EFF_TBV
  }
#else
}
#endif



//---------------------- FINAL CONTECH ---------------------------


uint64_t DynamicAnalysis::CalculateSpanFinal (int ResourceType)
{
  
  uint64_t
  Span = 0;
  
  //If there are instructions of this type....
  if (InstructionsCountExtended[ResourceType] > 0) {
    uint64_t
    Latency = ExecutionUnitsLatency[ResourceType];
    uint64_t
    First = FirstNonEmptyLevel[ResourceType];
    uint64_t
    DominantLevel = First;
    uint64_t
    LastCycle = LastIssueCycleVector[ResourceType];
    
    Span += Latency;
    
    //Start from next level to first non-emtpy level
    for (unsigned i = First + 1; i <= LastCycle; i += 1) {
      //Check whether there is instruction scheduled in this cycle!
      if (IsEmptyLevelFinal (ResourceType, i) == false) {
        if (DominantLevel + Latency != 0 && i <= DominantLevel + Latency - 1) {
          if (i + Latency > DominantLevel + Latency && Latency != 0) {
            Span += ((i + Latency) - max ((DominantLevel + Latency), (uint64_t) 1));
            DominantLevel = i;
          }
        }
        else {
          Span += Latency;
          DominantLevel = i;
        }
      }
    }
  }
  return Span;
}

bool
DynamicAnalysis::IsEmptyLevelFinal (unsigned ExecutionResource, uint64_t Level)
{
  if (ExecutionResource <= nExecutionUnits) {
    if (ACTFinal.get_node_ACT (Level, ExecutionResource)) {
      return false;
    }
  }
  
  
#ifdef EFF_TBV
  return !FullOccupancyCyclesTree[ExecutionResource].get_node (Level);
#else
  int TreeChunk = Level / SplitTreeRange;
  return !FullOccupancyCyclesTree[TreeChunk].get_node (Level, ExecutionResource);
#endif
}


unsigned
DynamicAnalysis::CalculateLatencyOnlySpanFinal(unsigned i){
  
  
  CLSFCache[i].resize(LastIssueCycleFinal + MaxLatencyResources);
  
  CLSFCache[i] |= CGSFCache[i];
  
 	CLSFCache[i] ^= CISFCache[i];
  
  return CLSFCache[i].count();
}



unsigned DynamicAnalysis::GetLatencyIssueOverlap(unsigned i){
  
  dynamic_bitset <> BitMesh (LastIssueCycleFinal + MaxLatencyResources);
  
  BitMesh ^= CISFCache[i];
  
  BitMesh &= CLSFCache[i];
  
  return BitMesh.count();
}






unsigned
DynamicAnalysis::GetGroupSpanFinal (vector < int >&ResourcesVector)
{
  
  dynamic_bitset <> BitMesh (LastIssueCycleFinal + MaxLatencyResources);
  for (size_t j = 0; j < ResourcesVector.size (); j++) {
    // Should probably recurse and calculate this value just in case
    if (CGSFCache[ResourcesVector[j]].size () != 0) {
      BitMesh |= CGSFCache[ResourcesVector[j]];
    }
  }
  return BitMesh.count ();
}

unsigned
DynamicAnalysis::GetGroupOverlapCyclesFinal (vector < int >&ResourcesVector)
{
  dynamic_bitset <> BitMesh (LastIssueCycleFinal + MaxLatencyResources);
  for (size_t j = 0; j < ResourcesVector.size (); j++) {
    // Should probably recurse and calculate this value just in case
    if (CGSFCache[ResourcesVector[j]].size () != 0 && CGSFCache[ResourcesVector[j]].count () != 0) {
      
      if (BitMesh.count() == 0) {
        BitMesh ^= CGSFCache[ResourcesVector[j]];
        
      }
      else {
 
        BitMesh &= CGSFCache[ResourcesVector[j]];
        if(BitMesh.count() == 0)
          return 0;
        //}
      }
    }
  }
  return BitMesh.count ();
}


// Calculate the total span of resources, and then do an AND with the
// span of the corresponding resource.
unsigned
DynamicAnalysis::GetOneToAllOverlapCyclesFinal (vector < int >&ResourcesVector)
{
  dynamic_bitset <> BitMesh (LastIssueCycleFinal + MaxLatencyResources);
  
  for (size_t j = 1; j < ResourcesVector.size (); j++) {
    if (CGSFCache[ResourcesVector[j]].size () != 0 && CGSFCache[ResourcesVector[j]].count () != 0)
      BitMesh |= CGSFCache[ResourcesVector[j]];
  }
  //We assume the first resoruce is the target resource, that is, the resource we
  // want to calculate the overlap with all others
  BitMesh &= CGSFCache[ResourcesVector[0]];
  
  return BitMesh.count ();
}


// Calculate the total span of resources, and then do an AND with the
// span of the corresponding resource.
unsigned
DynamicAnalysis::GetOneToAllOverlapCyclesFinal (vector < int >&ResourcesVector, bool Issue)
{
  dynamic_bitset <> BitMesh (LastIssueCycleFinal + MaxLatencyResources);
  
  for (size_t j = 1; j < ResourcesVector.size (); j++) {
    if (CGSFCache[ResourcesVector[j]].size () != 0 && CGSFCache[ResourcesVector[j]].count () != 0)
      BitMesh |= CGSFCache[ResourcesVector[j]];
  }
  
  if (Issue == true){
    BitMesh &= CISFCache[ResourcesVector[0]];
  }else{

    BitMesh &= CLSFCache[ResourcesVector[0]];
  }
  
  
  return BitMesh.count ();
}





unsigned
DynamicAnalysis::CalculateGroupSpanFinal (vector < int >&ResourcesVector)
{
  unsigned Span = 0;
  unsigned MaxLatency = 0;
  uint64_t First = 0;
  bool EmptyLevel = true;
  bool IsGap = false;
  int NResources = ResourcesVector.size ();
  uint64_t LastCycle = 0;
  uint64_t ResourceLastCycle = 0;
  unsigned MaxLatencyLevel = 0;
  unsigned ResourceType = 0;
  unsigned AccessWidth = 0;
  unsigned SpanIncrease = 0;
  
  for (unsigned i = 0; i < MAX_RESOURCE_VALUE; i++) {
    if (LastIssueCycleVector[i] > LastCycle)
      LastCycle = LastIssueCycleVector[i];
  }
  LastCycle += MaxLatencyResources;		// to be safe
  
  // Prepare a cache of values
  if (NResources == 1) {
    CGSFCache[ResourcesVector[0]].resize (LastCycle, false);
  }
  else {
    dynamic_bitset <> BitMesh (LastCycle);
    
    for (int j = 0; j < NResources; j++) {
      
      // Should probably recurse and calculate this value just in case
      if (CGSFCache[ResourcesVector[j]].size () == 0) {
        
        vector < int >tv;
        tv.push_back (ResourcesVector[j]);
        CalculateGroupSpanFinal (tv);
      }
      
      BitMesh |= CGSFCache[ResourcesVector[j]];
    }
    
    
    return BitMesh.count ();
  }
  
  LastCycle = 0;
  
  //Determine first non-empty level and LastCycle
  for (int j = 0; j < NResources; j++) {
    ResourceType = ResourcesVector[j];
    
    if (InstructionsCountExtended[ResourceType] > 0) {
      AccessWidth = AccessWidths[ResourceType];
      
      if (EmptyLevel == true) {	// This will be only executed the first time of a non-empty level
        EmptyLevel = false;
        First = FirstNonEmptyLevel[ResourceType];
        
        if (ExecutionUnitsThroughput[ResourceType] == INF) {
          MaxLatency = ExecutionUnitsLatency[ResourceType];
        }
        else
          MaxLatency =
          max (ExecutionUnitsLatency[ResourceType],
               (unsigned) ceil (AccessWidth / ExecutionUnitsThroughput[ResourceType]));
      }
      else {
        if (First == FirstNonEmptyLevel[ResourceType]) {
          if (ExecutionUnitsThroughput[ResourceType] == INF) {
            MaxLatency = max (MaxLatency, ExecutionUnitsLatency[ResourceType]);
          }
          else
            MaxLatency =
            max (MaxLatency,
                 max (ExecutionUnitsLatency[ResourceType],
                      (unsigned) ceil (AccessWidth / ExecutionUnitsThroughput[ResourceType])));
        }
        else {
          First = min (First, FirstNonEmptyLevel[ResourceType]);
          if (First == FirstNonEmptyLevel[ResourceType]) {
            if (ExecutionUnitsThroughput[ResourceType] == INF) {
              MaxLatency = ExecutionUnitsLatency[ResourceType];
            }
            else
              MaxLatency =
              max (ExecutionUnitsLatency[ResourceType],
                   (unsigned) ceil (AccessWidth / ExecutionUnitsThroughput[ResourceType]));
          }
        }
      }
      
      ResourceLastCycle = LastIssueCycleVector[ResourceType];
      LastCycle = max (LastCycle, ResourceLastCycle);
    }
  }
  
  
  
  unsigned DominantLevel = First;
  
  if (NResources == 1 && MaxLatency > 0) {
    //unsigned nBits = 0;
    for (unsigned q = 0; q < MaxLatency; q++) {
      CGSFCache[ResourcesVector[0]][First + q] = 1;
    }
  }
  
  if (EmptyLevel == false) {
    Span += MaxLatency;
    
    for (unsigned i = First + 1; i <= LastCycle; i++) {
      // For sure there is at least resource for which this level is not empty.
      //Determine MaxLatency of Level
      MaxLatencyLevel = 0;
      for (int j = 0; j < NResources; j++) {
        ResourceType = ResourcesVector[j];
        
        if (i <= LastIssueCycleVector[ResourceType]) {
          if (IsEmptyLevelFinal (ResourceType, i) == false) {
            IsGap = false;
            // MaxLatencyLevel = max(MaxLatencyLevel, GetInstructionLatency(ResourcesVector[j]));
            AccessWidth = AccessWidths[ResourceType];
            
            if (ExecutionUnitsThroughput[ResourceType] == INF) {
              MaxLatencyLevel = max (MaxLatencyLevel, ExecutionUnitsLatency[ResourceType]);
            }
            else {
              MaxLatencyLevel =
              max (MaxLatencyLevel,
                   max (ExecutionUnitsLatency[ResourceType],
                        (unsigned) ceil (AccessWidth / ExecutionUnitsThroughput[ResourceType])));
            }
          }
        }
      }
      
      //That is, only if there are instructions scheduled in this cycle
      if (MaxLatencyLevel != 0) {
        // Add the first condition because if Latency=0 is allowed, it can happen
        // that DominantLevel+MaxLatency-1 is a negative number, so the loop
        // is entered incorrectly.
        if (DominantLevel + MaxLatency != 0 && i <= DominantLevel + MaxLatency - 1) {
          if (i + MaxLatencyLevel > DominantLevel + MaxLatency && MaxLatencyLevel != 0) {
            SpanIncrease = ((i + MaxLatencyLevel) - max ((DominantLevel + MaxLatency), (unsigned) 1));
            Span += SpanIncrease;
            DominantLevel = i;
            MaxLatency = MaxLatencyLevel;
          }
        }
        else {
          SpanIncrease = MaxLatencyLevel;
          Span += MaxLatencyLevel;
          DominantLevel = i;
          MaxLatency = MaxLatencyLevel;
        }
      }
      else {
        if (i > DominantLevel + MaxLatency - 1) {
          if (NResources == 1 && IsGap == false) {
            SpanGaps[ResourceType]++;
            IsGap = true;
          }
        }
      }
      
      if (NResources == 1 && MaxLatencyLevel > 0) {
        for (unsigned q = 0; q < MaxLatencyLevel; q++) {
          CGSFCache[ResourcesVector[0]][i + q] = 1;
        }
      }
    }
  }
  
  // Delta should be 0
  unsigned delta = Span - CGSFCache[ResourcesVector[0]].count ();
  if (delta != 0) {
    report_fatal_error ("delta != 0\n");
    
  }
  if (delta != 0 && NResources == 1) {
    LastCycle = CGSFCache[ResourcesVector[0]].size () - MaxLatencyResources;
    for (; delta != 0; delta--) {
      CGSFCache[ResourcesVector[0]][LastCycle + delta] = 1;
    }
  }
  
  return Span;
}

unsigned
DynamicAnalysis::CalculateIssueSpanFinal (vector < int >&ResourcesVector)
{
  unsigned Span = 0;
  unsigned MaxLatency = 0;
  uint64_t First = 0;
  bool EmptyLevel = true;
  int NResources = ResourcesVector.size ();
  uint64_t LastCycle = 0;
  uint64_t ResourceLastCycle = 0;
  unsigned MaxLatencyLevel = 0;
  unsigned ResourceType = 0;
  unsigned AccessWidth = 0;
  unsigned TmpLatency = 0;
  
  for (unsigned i = 0; i < MAX_RESOURCE_VALUE; i++) {
    if (LastIssueCycleVector[i] > LastCycle)
      LastCycle = LastIssueCycleVector[i];
  }
  LastCycle += MaxLatencyResources;		// to be safe
  
  // Prepare a cache of values
  if (NResources == 1) {
    CISFCache[ResourcesVector[0]].resize (LastCycle, false);
  }
  else {
    dynamic_bitset <> BitMesh (LastCycle);
    
    for (int j = 0; j < NResources; j++) {
      // Should probably recurse and calculate this value just in case
      if (CISFCache[ResourcesVector[j]].size () == 0) {
        vector < int >tv;
        tv.push_back (ResourcesVector[j]);
        CalculateIssueSpanFinal (tv);
      }
      BitMesh |= CISFCache[ResourcesVector[j]];
    }
    
    return BitMesh.count ();
  }
  
  //Determine first non-empty level and LastCycle
  for (int j = 0; j < NResources; j++) {
    ResourceType = ResourcesVector[j];
    
    if (InstructionsCountExtended[ResourceType] > 0) {
      AccessWidth = AccessWidths[ResourceType];
      if (ExecutionUnitsThroughput[ResourceType] == INF)
        TmpLatency = 1;
      else
        TmpLatency = GetIssueCycleGranularity(ResourceType, AccessWidth, 1);
      if (EmptyLevel == true) {	// This will be only executed the first time of a non-empty level
        EmptyLevel = false;
        First = FirstNonEmptyLevel[ResourceType];
        MaxLatency = TmpLatency;
      }
      else {
        if (First == FirstNonEmptyLevel[ResourceType])
          MaxLatency = max (MaxLatency, TmpLatency);
        else {
          First = min (First, FirstNonEmptyLevel[ResourceType]);
          if (First == FirstNonEmptyLevel[ResourceType])
            MaxLatency = TmpLatency;
        }
      }
      ResourceLastCycle = LastIssueCycleVector[ResourceType];
      
      LastCycle = max (LastCycle, ResourceLastCycle);
    }
  }
  
  
  
  unsigned DominantLevel = First;
  if (NResources == 1 && MaxLatency > 0) {
    //unsigned nBits = 0;
    for (unsigned q = 0; q < MaxLatency; q++) {
      
      CISFCache[ResourcesVector[0]][First + q] = 1;
    }
  }
  
  if (EmptyLevel == false) {
    Span += MaxLatency;
    
    //Start from next level to first non-empty level
    for (unsigned i = First + 1; i <= LastCycle; i++) {
      //Determine MaxLatency of Level
      MaxLatencyLevel = 0;
      for (int j = 0; j < NResources; j++) {
        ResourceType = ResourcesVector[j];
        
        if (i <= LastIssueCycleVector[ResourceType]) {
          if (IsEmptyLevelFinal (ResourceType, i) == false) {
            
            AccessWidth = AccessWidths[ResourceType];
            if (ExecutionUnitsThroughput[ResourceType] == INF)
              TmpLatency = 1;
            else
              TmpLatency = GetIssueCycleGranularity(ResourceType, AccessWidth, 1);
            MaxLatencyLevel = max (MaxLatencyLevel, TmpLatency);
          }
        }
      }
      
      
      //That is, only if there are instructions scheduled in this cycle
      if (MaxLatencyLevel != 0) {
        if (i <= DominantLevel + MaxLatency - 1) {
          if (i + MaxLatencyLevel > DominantLevel + MaxLatency && MaxLatencyLevel != 0) {
            Span += ((i + MaxLatencyLevel) - max ((DominantLevel + MaxLatency), (unsigned) 1));
            DominantLevel = i;
            MaxLatency = MaxLatencyLevel;
          }
        }
        else {
          Span += MaxLatencyLevel;
          DominantLevel = i;
          MaxLatency = MaxLatencyLevel;
        }
      }
      
      if (NResources == 1 && MaxLatencyLevel > 0) {
        for (unsigned q = 0; q < MaxLatencyLevel; q++) {
          
          CISFCache[ResourcesVector[0]][i + q] = 1;
        }
      }
    }
  }
  
  assert (CISFCache[ResourcesVector[0]].count () == Span);
  
  return Span;
}

// Tree is unbalanced, switch from recursive to iterative method
void
DynamicAnalysis::ComputeAvailableTreeFinalHelper (uint p, Tree < uint64_t > *t, uint d)
{
  uint64_t lastKey = 0;
  
  while (true) {
    if (t->left != NULL) {
      t->left->prev = t;
      t = t->left;
      continue;
    }
    // insert element
    if (t->key >= lastKey) {
      ACTNode *n = new ACTNode;
      
      n->key = t->key;
      n->issueOccupancy = t->issueOccupancy;
      n->widthOccupancy = t->widthOccupancy;
      n->occupancyPrefetch = t->occupancyPrefetch;
      n->address = t->address;
      ACTFinal.push_back (n, p);
      
      lastKey = t->key;
    }
    
    if (t->right != NULL) {
      t->right->prev = t;
      t = t->right;
      continue;
    }
    
    if (t->prev != NULL) {
      Tree < uint64_t > *old = t;
      t = t->prev;
      if (t->left == old)
        t->left = NULL;
      if (t->right == old)
        t->right = NULL;
      delete old;
      continue;
    }
    else {
      break;
    }
  }
}

void
DynamicAnalysis::ComputeAvailableTreeFinal ()
{
  uint p = 0;
  
  for (auto it = AvailableCyclesTree.begin (), et = AvailableCyclesTree.end (); it != et; ++it) {
    if ((*it) != NULL) {
      DEBUG (dbgs () << "ACT convert on " << p << "\t" << (*it)->size << "\t");
      ComputeAvailableTreeFinalHelper (p, *it, 0);
      *it = NULL;
    }
    p++;
  }
}

void
ACT::push_back (ACTNode * n, unsigned BitPosition)
{
  uint64_t i = n->key;
  uint64_t TreeChunk = i / SplitTreeRange;
  if (TreeChunk >= act_vec.size ()) {
    act_vec.resize (TreeChunk + 1);
  }
  
  bool cond = (n->issueOccupancy != 0);	// Add optional prefetch conditional
  if (cond)
    act_vec[TreeChunk].insert_node (n->key, BitPosition);
  
  delete n;
}

bool
ACT::get_node_ACT (uint64_t key, unsigned BitPosition)
{
  uint64_t TreeChunk = key / SplitTreeRange;
  if (TreeChunk >= act_vec.size ()) {
    return false;
  }
  
  return act_vec[TreeChunk].get_node (key, BitPosition);
}

size_t
ACT::size ()
{
  //return act_map.size();
  return 0;
}

void
ACT::clear ()
{
  act_vec.clear ();
}


//===----------------------------------------------------------------------===//
//                        Print statistics
//===----------------------------------------------------------------------===//

void
DynamicAnalysis::printHeaderStat (string Header)
{
  
  dbgs () << "//===--------------------------------------------------------------===//\n";
  dbgs () << "//                     " << Header << "                                    \n";
  dbgs () << "//===--------------------------------------------------------------===//\n";
}






void
DynamicAnalysis::finishAnalysisContechSimplified ()
{
  
  unsigned long long TotalSpan = 0;
  uint64_t TotalStallSpan = 0;
  float Performance = 0;
  uint64_t Total;
  uint64_t T1, T2, OverlapCycles, MinResourceSpan, MinResource;
  vector < int >compResources;
  vector < int >movResources;
  vector < int >memResources;
  vector < uint64_t > ResourcesSpan (nExecutionUnits + nPorts + nAGUs + nLoadAGUs + nStoreAGUs + nBuffers);
  vector < uint64_t > ResourcesTotalStallSpanVector (nExecutionUnits + nPorts + nAGUs + nLoadAGUs + nStoreAGUs + nBuffers);
  vector < vector < uint64_t > >ResourcesResourcesNoStallSpanVector (nExecutionUnits, vector < uint64_t > (nExecutionUnits));
  vector < vector < uint64_t > >ResourcesResourcesSpanVector (nExecutionUnits, vector < uint64_t > (nExecutionUnits));
  vector < vector < uint64_t > >ResourcesStallSpanVector (nExecutionUnits, vector < uint64_t > (nExecutionUnits));
  vector < vector < uint64_t > >StallStallSpanVector (nBuffers, vector < uint64_t > (nBuffers));
  vector < vector < uint64_t > >ResourcesIssueStallSpanVector (nExecutionUnits, vector < uint64_t > (nBuffers));
  
  
  
  
  
#ifdef DEBUG_OOO_BUFFERS
  DEBUG (dbgs () << "Starting while loop\n");
  DEBUG (dbgs () << "Sise of RS " << ReservationStationIssueCycles.size () << "\n");
  DEBUG (dbgs () << "Sise of ROB " << ReorderBufferCompletionCycles.size () << "\n");
  if(SmallBuffers)
    DEBUG (dbgs () << "Sise of LB " << LoadBufferCompletionCycles.size() << "\n");
  else
    DEBUG (dbgs () << "Sise of LB " << node_size(LoadBufferCompletionCyclesTree) << "\n");
  DEBUG (dbgs () << "Sise of SB " << StoreBufferCompletionCycles.size () << "\n");
  DEBUG (dbgs () << "Sise of LFB " << LineFillBufferCompletionCycles.size () << "\n");
  DEBUG (dbgs () << "______________________________________________________\n");
#endif
  bool BufferNonEmpty=false;
  if(SmallBuffers){
    if (LoadBufferCompletionCycles.size() != 0)
      BufferNonEmpty = true;
  }else{
    if (node_size(LoadBufferCompletionCyclesTree) != 0)
      BufferNonEmpty = true;
  }
  
  
  // Increase FetchCycle until all buffers are empty
  while (ReservationStationIssueCycles.size () != 0
         || ReorderBufferCompletionCycles.size () != 0
         || BufferNonEmpty || StoreBufferCompletionCycles.size () != 0
         || LineFillBufferCompletionCycles.size () != 0) {
    // In IncreaseInstructionFetchCycle(), InstructionFetchCycle only increases when
    // RS or ROB are full. But in this case, they may not get full, but we just
    // want to empty them
    // We don't increase fetch cycle here anymore because it is increased in the
    // function IncreaseInstructionFetchCycle() by setting the argument to true
    //  InstructionFetchCycle++;
    IncreaseInstructionFetchCycle (true);
    
#ifdef DEBUG_OOO_BUFFERS
    
    PrintReservationStation ();
    PrintReorderBuffer ();
    PrintStoreBuffer ();
    if(SmallBuffers)
      PrintLoadBuffer();
    else
      PrintLoadBufferTree ();
    
    PrintLineFillBuffer ();
    
    DEBUG (dbgs () << "Size of RS " << ReservationStationIssueCycles.size () << "\n");
    DEBUG (dbgs () << "Size of ROB " << ReorderBufferCompletionCycles.size () << "\n");
    if(SmallBuffers)
      DEBUG (dbgs () << "Sise of LB " << LoadBufferCompletionCycles.size() << "\n");
    else
      DEBUG (dbgs () << "Sise of LB " << node_size(LoadBufferCompletionCyclesTree) << "\n");
    DEBUG (dbgs () << "Size of SB " << StoreBufferCompletionCycles.size () << "\n");
    DEBUG (dbgs () << "Size of LFB " << LineFillBufferCompletionCycles.size () << "\n");
    DEBUG (dbgs () << "______________________________________________________\n");
#endif
    BufferNonEmpty=false;
    //Update Buffer Size
    if(SmallBuffers){
      if (LoadBufferCompletionCycles.size() != 0)
        BufferNonEmpty = true;
    }else{
      if (node_size(LoadBufferCompletionCyclesTree) != 0)
        BufferNonEmpty = true;
    }
    
    
  }
  
  
  
#ifdef DEBUG_OOO_BUFFERS
  
  DEBUG (dbgs () << "Size of RS " << ReservationStationIssueCycles.size () << "\n");
  DEBUG (dbgs () << "Size of ROB " << ReorderBufferCompletionCycles.size () << "\n");
  if(SmallBuffers)
    DEBUG (dbgs () << "Sise of LB " << LoadBufferCompletionCycles.size() << "\n");
  else
    DEBUG (dbgs () << "Sise of LB " << node_size(LoadBufferCompletionCyclesTree) << "\n");
  DEBUG (dbgs () << "Size of SB " << StoreBufferCompletionCycles.size () << "\n");
  DEBUG (dbgs () << "Size of LFB " << LineFillBufferCompletionCycles.size () << "\n");
#endif
  
  
  
  for (unsigned i = 0; i < nArithmeticNodes; i++)
    compResources.push_back (i);
  for (unsigned i = nArithmeticNodes; i < nArithmeticNodes + nMovNodes; i++)
    movResources.push_back (i);
  
  for (unsigned i = nArithmeticNodes + nMovNodes; i < nArithmeticNodes + nMovNodes + nMemNodes; i++)
    memResources.push_back (i);
  
  
  for (unsigned j = 0; j < nExecutionUnits + nAGUs + nPorts + nBuffers; j++) {
    LastIssueCycleVector.push_back (GetLastIssueCycle (j));
    
    // If all buffers sizes are infinity, or a buffer does not get full, we don't
    // increment in the previous while loop InstructionFetchCycle. So this check
    // only makes sense when RS or ROB have limited size (because they hold
    // all type of instructions until they are issued)
    if (InstructionFetchCycle != 0 && LastIssueCycleVector[j] > InstructionFetchCycle
        && ReservationStationIssueCycles.size () != 0 && ReorderBufferCompletionCycles.size () != 0) {
      report_fatal_error ("LastIssueCycle > InstructionFetchCycle for resource\n");
    }
  }
  
  ComputeAvailableTreeFinal ();
  
  
  for (unsigned i = 0; i < MAX_RESOURCE_VALUE; i++) {
    vector < int >tv;
    if (LastIssueCycleVector[i] > LastIssueCycleFinal)
      LastIssueCycleFinal = LastIssueCycleVector[i];
    if (InstructionsCountExtended[i] != 0) {
      tv.push_back (i);
      
      ResourcesSpan[i] = CalculateGroupSpanFinal (tv);
      if (ExecutionUnitsLatency[i]!=0)
        IssueSpan[i] = CalculateIssueSpanFinal (tv);
      else
        IssueSpan[i] = 0;
      //  LatencySpan[i] = CalculateLatencySpanFinal(i);
#ifdef ASSERT
      if (i >= RS_STALL && i <= LFB_STALL) {
        Span = InstructionsCountExtended[i];
        uint64_t CalculateSpanResult = CalculateSpanFinal (i);
        // All three methods should give the same span
        if (!(CalculateSpanResult == Span && Span == ResourcesSpan[i]
              && ResourcesSpan[i] == IssueSpan[i]))
          report_fatal_error ("Spans differ: Span (" + Twine (Span) + "), CalculateSpan (" + Twine (CalculateSpanResult) +
                              "), CalculateGroupSpan (" + Twine (ResourcesSpan[i]) + ")");
        
      }
      
#endif
    }
  }
  
  for (unsigned i = 0; i < MAX_RESOURCE_VALUE; i++) {
    if (InstructionsCountExtended[i] != 0 && ExecutionUnitsLatency[i]!=0) {
      LatencyOnlySpan[i] = CalculateLatencyOnlySpanFinal(i);
    }
  }
  

  //================= Calculate total span ==========================//
  
  
  unsigned long long InstructionLatency = 0;
  uint64_t LastCycle = 0;
  
  for (unsigned j = 0; j < nExecutionUnits; j++) {
    // If there are instructions of this type
    if (InstructionsCountExtended[j] > 0) {
      InstructionLatency = ExecutionUnitsLatency[j];
      
      LastCycle = LastIssueCycleVector[j];
      TotalSpan = max (LastCycle + InstructionLatency, TotalSpan);
    }
  }
  
  
  if (ReportOnlyPerformance) {
    
    dbgs () << "TOTAL FLOPS" << "\t" << InstructionsCount[0] << "\t\t" << CalculateGroupSpanFinal (compResources) << " \n";
    dbgs () << "TOTAL MOPS" << "\t" << InstructionsCount[1] << "\t\t" << CalculateGroupSpanFinal (memResources) << " \n";
    dbgs () << "TOTAL" << "\t\t" << InstructionsCount[0] + InstructionsCount[1] << "\t\t" << TotalSpan << " \n";
    Performance = (float) InstructionsCount[0] / ((float) TotalSpan);
    fprintf (stderr, "PERFORMANCE %1.3f\n", Performance);
    
    
    return;
  }
  
  //Reuse Distance Distribution
  printHeaderStat ("Reuse Distance distribution");
  
  map < int, int >::iterator ReuseDistanceMapIt;
  for (ReuseDistanceMapIt = ReuseDistanceDistribution.begin (); ReuseDistanceMapIt != ReuseDistanceDistribution.end ();
       ++ReuseDistanceMapIt) {
    dbgs () << ReuseDistanceMapIt->first << " " << ReuseDistanceMapIt->second << "\n";
  }
  dbgs () << "DATA_SET_SIZE\t" << node_size (ReuseTree) << "\n";
  
  printHeaderStat ("Statistics");
  
  
  
  // Calculate Resources-total stall span
  
  {

    for (unsigned i = 0; i < nExecutionUnits; i++) {
      vector < int >tv;

      if (InstructionsCountExtended[i] != 0) {
        tv.push_back (i);
        
        for (uint j = RS_STALL; j <= LFB_STALL; j++) {
          if (InstructionsCountExtended[j] != 0) {
            
            tv.push_back (j);
            
          }
        }
        ResourcesTotalStallSpanVector[i] = GetGroupSpanFinal (tv);
        
      }
    }
  }
  
  
  
  //==================== Print resource statistics =============================//
  
  dbgs () << "RESOURCE\tN_OPS_ISSUED\tSPAN\t\tISSUE-SPAN\tSTALL-SPAN\t\tMAX_OCCUPANCY\n";
  
  for (unsigned j = 0; j < nExecutionUnits; j++) {
    
    {
      
      dbgs () << GetResourceName (j) << "\t\t" <<
      InstructionsCountExtended[j] << "\t\t" << ResourcesSpan[j] << "\t\t" << IssueSpan[j] << "\t\t" <<
      ResourcesTotalStallSpanVector[j] << "\t\t" <<MaxOccupancy[j] << " \n";
      
    }
  }
  
  
  //==================== Print stall cycles =============================//
  
  printHeaderStat ("Stall Cycles");
  
  
  
  dbgs () << "RESOURCE\tN_STALL_CYCLES\t\tAVERAGE_OCCUPANCY\t\tFRACTION_OCCUPANCY\n";
  
  for (int j = RS_STALL; j <= LFB_STALL; j++) {
    if (TotalSpan == 0) {
      dbgs () << GetResourceName (j) << "\t\t" << ResourcesSpan[j] << "\t\t" << INF << "\n";
    }
    else {
      
      double AverageOccupancy = BuffersOccupancy[j -RS_STALL] /(double)TotalSpan;
      double FractionOccupancy = 0.0;
      dbgs () << GetResourceName (j) << "\t\t" << ResourcesSpan[j] << "\t\t";
      fprintf (stderr, " %1.3f\t\t", AverageOccupancy);
      switch(j){
        case RS_STALL:
          if (ReservationStationSize!= 0)
            FractionOccupancy =  AverageOccupancy*100/ReservationStationSize;
          break;
        case ROB_STALL:
          if (ReorderBufferSize != 0)
            FractionOccupancy =  AverageOccupancy*100/ReorderBufferSize;
          break;
        case LB_STALL:
          if (LoadBufferSize != 0)
            FractionOccupancy =  AverageOccupancy*100/LoadBufferSize;
          break;
        case SB_STALL:
          if(StoreBufferSize!= 0)
            FractionOccupancy =  AverageOccupancy*100/StoreBufferSize;
          break;
        case LFB_STALL:
          if (LineFillBufferSize!= 0)
            FractionOccupancy =  AverageOccupancy*100/LineFillBufferSize;
          break;
        default:
          report_fatal_error ("Buffer not recognized");
          
      }
      fprintf (stderr, " %1.3f\n", FractionOccupancy);
      
      
      
      
      
    }
  }
  
  
  printHeaderStat ("Span Only Stalls");
  
  {
    vector < int >tv;
    
    for (unsigned i = RS_STALL; i <= LFB_STALL; i++) {
      if (InstructionsCountExtended[i] > 0) {
        // This TotalStallSpan is just in case there are only stalls from one buffer
        TotalStallSpan = ResourcesSpan[i];
        tv.push_back (i);
      }
    }
    if (tv.empty () == true) {
      TotalStallSpan = 0;
    }
    else {
      
      TotalStallSpan = GetGroupSpanFinal (tv);
      
    }
  }
  
  dbgs () << TotalStallSpan << "\n";
  
  
  //==================== Print port Occupancy =============================//
  if(ConstraintPorts){
    printHeaderStat ("Port occupancy");
    
    dbgs () << "PORT\t\tDISPATCH CYCLES\n";
    
    {
      
      //for (int j = PORT_0; j <= PORT_5; j++) {
      for (unsigned j = 0; j <nPorts; j++) {
        dbgs () << GetResourceName (PORT_0+j) << "\t\t" << ResourcesSpan[PORT_0+j] << "\n";
      }
    }
  }
  //==================== Resource-Stall Span =============================//
  
  printHeaderStat ("Resource-Stall Span");
  dbgs () << "RESOURCE";
  for (int j = RS_STALL; j <= LFB_STALL; j++) {
    dbgs () << "\t" << GetResourceName (j);
  }
  dbgs () << "\n";
  {
    
    for (unsigned i = 0; i < nExecutionUnits; i++) {
      
      dbgs () << GetResourceName (i) << "\t\t";
      for (uint j = RS_STALL; j <= LFB_STALL; j++) {
        vector < int >tv;
        
        if (InstructionsCountExtended[i] != 0 && InstructionsCountExtended[j] != 0) {
          tv.push_back (i);
          tv.push_back (j);
          ResourcesStallSpanVector[i][j - RS_STALL] = GetGroupSpanFinal (tv);
        }
        else {
          if (InstructionsCountExtended[i] == 0) {
            ResourcesStallSpanVector[i][j - RS_STALL] = InstructionsCountExtended[j];
          }
          else {
            if (InstructionsCountExtended[j] == 0) {
              ResourcesStallSpanVector[i][j - RS_STALL] = ResourcesSpan[i];
            }
          }
        }
        dbgs () << ResourcesStallSpanVector[i][j - RS_STALL] << "\t";
      }
      dbgs () << "\n";
    }
  }
  
  //==================== Resource-Stall Overlap =============================//
  
  printHeaderStat ("Resource-Stall Overlap (0-1)");
  dbgs () << "RESOURCE";
  for (unsigned j = RS_STALL; j <= LFB_STALL; j++) {
    dbgs () << "\t" << GetResourceName (j);
  }
  dbgs () << "\n";
  
  float OverlapPercetage;
  for (unsigned i = 0; i < nExecutionUnits; i++) {
    dbgs () << GetResourceName (i) << "\t\t";
    for (uint j = RS_STALL; j <= LFB_STALL; j++) {
      if (InstructionsCountExtended[i] != 0 && InstructionsCountExtended[j] != 0 && ResourcesSpan[i] != 0
          && ResourcesSpan[j] != 0) {
        Total = ResourcesStallSpanVector[i][j - RS_STALL];
        // When latency is zero, ResourcesSpan is zero. However, IssueSpan
        // might not be zero.
        T1 = ResourcesSpan[i];
        T2 = ResourcesSpan[j];
        assert (Total <= T1 + T2);
        OverlapCycles = T1 + T2 - Total;
        OverlapPercetage = (float) OverlapCycles / (float (min (T1, T2)));
        if (OverlapPercetage > 1.0) {
          report_fatal_error ("Overlap > 1.0 R-S overlap (0-1)");
        }
      }
      else {
        OverlapPercetage = 0;
      }
      fprintf (stderr, " %1.3f ", OverlapPercetage);
    }
    dbgs () << "\n";
  }
  
  //==================== ResourceIssue-Stall Span =============================//
  
  printHeaderStat ("ResourceIssue-Stall Span");
  dbgs () << "RESOURCE";
  for (unsigned j = RS_STALL; j <= LFB_STALL; j++) {
    dbgs () << "\t" << GetResourceName (j);
  }
  dbgs () << "\n";
  {
    for (unsigned i = 0; i < nExecutionUnits; i++) {
      dbgs () << GetResourceName (i) << "\t\t";
      for (uint j = RS_STALL; j <= LFB_STALL; j++) {
        dynamic_bitset <> BitMesh (LastIssueCycleFinal + MaxLatencyResources);
        
        if (InstructionsCountExtended[i] != 0 && InstructionsCountExtended[j] != 0 && ExecutionUnitsLatency[i]!= 0 && ExecutionUnitsLatency[j]!=0) {
          BitMesh |= CISFCache[i];
          BitMesh |= CGSFCache[j];
          ResourcesIssueStallSpanVector[i][j - RS_STALL] = BitMesh.count ();
        }
        else {
          if (InstructionsCountExtended[i] == 0 || ExecutionUnitsLatency[i]==0) {
            ResourcesIssueStallSpanVector[i][j - RS_STALL] = InstructionsCountExtended[j];;
          }
          else {
            if (InstructionsCountExtended[j] == 0 || ExecutionUnitsLatency[j]==0) {
              ResourcesIssueStallSpanVector[i][j - RS_STALL] = IssueSpan[i];;
            }
          }
        }
        dbgs () << ResourcesIssueStallSpanVector[i][j - RS_STALL] << "\t";
      }
      dbgs () << "\n";
    }
  }
  
  //==================== ResourceIssue-Stall Overlap =============================//
  
  printHeaderStat ("ResourceIssue-Stall Overlap (0-1)");
  dbgs () << "RESOURCE";
  for (unsigned j = RS_STALL; j <= LFB_STALL; j++) {
    dbgs () << "\t" << GetResourceName (j);
  }
  dbgs () << "\n";
  float OverlapPercentage;
  
  for (unsigned i = 0; i < nExecutionUnits; i++) {
    dbgs () << GetResourceName (i) << "\t\t";
    for (uint j = RS_STALL; j <= LFB_STALL; j++) {
      if (InstructionsCountExtended[i] != 0 && InstructionsCountExtended[j] != 0 && ExecutionUnitsLatency[i]!= 0 && ExecutionUnitsLatency[j]!=0) {
        Total = ResourcesIssueStallSpanVector[i][j - RS_STALL];
        T1 = IssueSpan[i];
        T2 = InstructionsCountExtended[j];
        assert (Total <= T1 + T2);
        OverlapCycles = T1 + T2 - Total;
        OverlapPercentage = (float) OverlapCycles / (float (min (T1, T2)));
        if (OverlapPercentage > 1.0) {
          report_fatal_error ("Overlap > 1.0 RI-S Overlap (0-1)");
        }
      }
      else {
        OverlapPercentage = 0;
      }
      fprintf (stderr, " %1.3f ", OverlapPercentage);
    }
    dbgs () << "\n";
  }
  
  //==================== Resource-Resource Span =============================//
  
  printHeaderStat ("Resource-Resource Span (resources span without stalls)");
  
  dbgs () << "RESOURCE";
  for (unsigned j = 0; j < nExecutionUnits; j++) {
    dbgs () << "\t" << GetResourceName (j);
  }
  dbgs () << "\n";
  
  {
    for (unsigned j = 1; j < nExecutionUnits; j++) {
      dbgs () << GetResourceName (j) << "\t\t";
      for (unsigned i = 0; i < j; i++) {
        vector < int >tv;
        
        if (InstructionsCountExtended[i] != 0 && InstructionsCountExtended[j] != 0 && ExecutionUnitsLatency[i]!= 0 && ExecutionUnitsLatency[j]!=0) {
          tv.push_back (j);
          tv.push_back (i);
          
          ResourcesResourcesNoStallSpanVector[j][i] = GetGroupSpanFinal (tv);
        }
        else {
          if (InstructionsCountExtended[i] == 0 || ExecutionUnitsLatency[i]==0) {
            ResourcesResourcesNoStallSpanVector[j][i] = ResourcesSpan[j];
          }
          else if (InstructionsCountExtended[j] == 0|| ExecutionUnitsLatency[j]==0) {
            ResourcesResourcesNoStallSpanVector[j][i] = ResourcesSpan[i];
          }
        }
        
        dbgs () << ResourcesResourcesNoStallSpanVector[j][i] << "\t";
      }
      dbgs () << "\n";
    }
  }
  
  
  printHeaderStat ("Resource-Resource Overlap Percentage (resources span without stall)");
  
  dbgs () << "RESOURCE";
  for (unsigned j = 0; j < nExecutionUnits; j++) {
    dbgs () << "\t" << GetResourceName (j);
  }
  dbgs () << "\n";
  
  for (unsigned j = 0; j < nExecutionUnits; j++) {
    dbgs () << GetResourceName (j) << "\t\t";
    for (unsigned i = 0; i < j; i++) {
      if (InstructionsCountExtended[i] != 0 && InstructionsCountExtended[j] != 0 && ResourcesSpan[j] != 0
          && ResourcesSpan[i] != 0) {
        Total = ResourcesResourcesNoStallSpanVector[j][i];
        T1 = ResourcesSpan[j];
        T2 = ResourcesSpan[i];
        OverlapCycles = T1 + T2 - Total;
        
        OverlapPercetage = (float) OverlapCycles / (float (min (T1, T2)));
        if (OverlapPercetage > 1.0) {
          report_fatal_error ("Overlap > 1.0 R-R overlap % (resources span without stall)");
        }
      }
      else {
        OverlapPercetage = 0;
      }
      fprintf (stderr, " %1.3f ", OverlapPercetage);
    }
    dbgs () << "\n";
  }
  
  //==================== Resource-Resource Span (with stalls) =============================//
  
  printHeaderStat ("Resource-Resource Span (resources span with stalls)");
  
  dbgs () << "RESOURCE";
  for (unsigned j = 0; j < nExecutionUnits; j++) {
    dbgs () << "\t" << GetResourceName (j);
  }
  dbgs () << "\n";
  {
    
    
    for (unsigned j = 0; j < nExecutionUnits; j++) {
      
      dbgs () << GetResourceName (j) << "\t\t";
      for (unsigned i = 0; i < j; i++) {
        if (InstructionsCountExtended[i] != 0 && InstructionsCountExtended[j] != 0 && ResourcesSpan[j] != 0
            && ResourcesSpan[i] != 0) {
          vector < int >tv;
          tv.push_back (j);
          tv.push_back (i);
          for (unsigned k = RS_STALL; k <= LFB_STALL; k++) {
            tv.push_back (k);
          }
          
          
          
          ResourcesResourcesSpanVector[j][i] = GetGroupSpanFinal (tv);
        }
        else {
          if (InstructionsCountExtended[i] == 0 || ResourcesSpan[i] == 0) {
            ResourcesResourcesSpanVector[j][i] = TotalStallSpan;
          }
          else if (InstructionsCountExtended[j] == 0 || ResourcesSpan[j] == 0) {
            ResourcesResourcesSpanVector[j][i] = ResourcesTotalStallSpanVector[i];
          }
        }
        
        dbgs () << ResourcesResourcesSpanVector[j][i] << "\t";
      }
      dbgs () << "\n";
    }
  }
  
  printHeaderStat ("Resource-Resource Overlap Percentage (resources span with stall)");
  
  dbgs () << "RESOURCE";
  for (unsigned j = 0; j < nExecutionUnits; j++) {
    dbgs () << "\t" << GetResourceName (j);
  }
  dbgs () << "\n";
  
  for (unsigned j = 0; j < nExecutionUnits; j++) {
    dbgs () << GetResourceName (j) << "\t\t";
    for (unsigned i = 0; i < j; i++) {
      if (InstructionsCountExtended[i] != 0 && IssueSpan[i]!= 0 && InstructionsCountExtended[j] != 0 && IssueSpan[j]!= 0 && ResourcesTotalStallSpanVector[j] != 0
          && ResourcesTotalStallSpanVector[i] != 0) {
        Total = ResourcesResourcesSpanVector[j][i];
        T1 = ResourcesTotalStallSpanVector[j];
        T2 = ResourcesTotalStallSpanVector[i];
        
        assert (Total <= T1 + T2);
        OverlapCycles = T1 + T2 - Total;
        OverlapPercetage = (float) OverlapCycles / (float (min (T1, T2)));
        if (OverlapPercetage > 1.0) {
          report_fatal_error ("Overlap > 1.0 R-R overlap % (resources span with stall)");
        }
      }
      else {
        OverlapPercetage = 0;
      }
      fprintf (stderr, " %1.3f ", OverlapPercetage);
    }
    dbgs () << "\n";
  }
  
  
  printHeaderStat ("Stall-Stall Span");
  
  dbgs () << "RESOURCE";
  for (unsigned j = RS_STALL; j <= LFB_STALL; j++) {
    dbgs () << "\t" << GetResourceName (j);
  }
  dbgs () << "\n";
  {
    for (unsigned j = RS_STALL; j <= LFB_STALL; j++) {
      dbgs () << GetResourceName (j) << "\t\t";
      for (unsigned i = RS_STALL; i < j; i++) {
        if (InstructionsCountExtended[j] != 0 && InstructionsCountExtended[i] != 0) {
          vector < int >tv;
          tv.push_back (j);
          
          tv.push_back (i);
          StallStallSpanVector[j - RS_STALL][i - RS_STALL] = GetGroupSpanFinal (tv);
        }
        else {
          if (InstructionsCountExtended[i] == 0) {
            StallStallSpanVector[j - RS_STALL][i - RS_STALL] = ResourcesSpan[j];
          }
          else if (InstructionsCountExtended[j] == 0) {
            StallStallSpanVector[j - RS_STALL][i - RS_STALL] = ResourcesSpan[i];
          }
        }
        dbgs () << StallStallSpanVector[j - RS_STALL][i - RS_STALL] << "\t";
      }
      dbgs () << "\n";
    }
  }
  
  printHeaderStat ("Stall-Stall Overlap Percentage ");
  
  dbgs () << "RESOURCE";
  for (unsigned j = RS_STALL; j <= LFB_STALL; j++) {
    dbgs () << "\t" << GetResourceName (j);
  }
  dbgs () << "\n";
  
  for (unsigned j = RS_STALL; j <= LFB_STALL; j++) {
    dbgs () << GetResourceName (j) << "\t\t";
    for (unsigned i = RS_STALL; i < j; i++) {
      if (InstructionsCountExtended[j] != 0 && InstructionsCountExtended[i] != 0) {
        Total = StallStallSpanVector[j - RS_STALL][i - RS_STALL];
        T1 = ResourcesSpan[j];
        T2 = ResourcesSpan[i];
        assert (Total <= T1 + T2);
        OverlapCycles = T1 + T2 - Total;
        OverlapPercetage = (float) OverlapCycles / (float (min (T1, T2)));
        
      }
      else {
        OverlapPercetage = 0;
      }
      fprintf (stderr, " %1.3f ", OverlapPercetage);
    }
    dbgs () << "\n";
  }
  
  
  
  // ===================== ALL OVERLAPS - SECOND APPROACH ===========================//
  {
    
    printHeaderStat ("All overlaps");
    unsigned n = nExecutionUnits+nBuffers;
    int nCombinations = 0;
    bool ResourceWithNoInstructions = false;
    
    // The variable k denotes the size of the groups
    for (unsigned k = 2; k <= nExecutionUnits+nBuffers; k++) {
      vector < vector < int > >combinations;
      vector < int >selected;
      vector < int >selector (n);
      
      
      fill (selector.begin (), selector.begin () + k, 1);
      do {
        for (unsigned i = 0; i < n; i++) {
          if (selector[i]) {
            selected.push_back (i);
          }
        }
        nCombinations++;
        std::vector < int >result (selected.size ());
        copy (selected.begin (), selected.end (), result.begin ());
        OverlapCycles = 0;
        MinResourceSpan = 0;
        OverlapPercetage = 0;
        MinResource = 0;
        
        for (unsigned j = 0; j < result.size (); j++) {
          if (InstructionsCountExtended[result[j]] == 0) {
            ResourceWithNoInstructions = true;
            break;
          }
        }
        if (ResourceWithNoInstructions== false) {
          
          for (unsigned j = 0; j < result.size (); j++) {
            dbgs () << result[j] << " ";
            if (InstructionsCountExtended[result[j]] > 0) {
              if (MinResourceSpan == 0){
                MinResourceSpan = ResourcesSpan[result[j]];
                MinResource =result[j];
              }
              else{
                MinResourceSpan = min (MinResourceSpan, ResourcesSpan[result[j]]);
                if(MinResourceSpan == ResourcesSpan[result[j]] )
                  MinResource = result[j];
              }
            }
          }
          
          if (MinResourceSpan != 0) {
            OverlapCycles = GetGroupOverlapCyclesFinal (result);
            OverlapPercetage = (float) OverlapCycles / (float (MinResourceSpan));
            for (unsigned j = 0; j < result.size (); j++) {
              OverlapsMetrics[result[j]]+= OverlapCycles;
            }
          }
          dbgs() << "| " << MinResource << " | ";
          dbgs () << OverlapCycles << " ";
          fprintf (stderr, " %1.3f\n", OverlapPercetage);
          
          OverlapsDerivatives[MinResource]+= OverlapPercetage*pow(-1,k+1);
          
          for (unsigned j = 0; j < result.size (); j++) {
            AverageOverlapsCycles[result[j]]+= OverlapCycles;
            AverageOverlaps[result[j]]+= OverlapPercetage;
            OverlapsCount[result[j]]++;
          }
          
          
          
          if (OverlapPercetage > 1.0) {
            dbgs () << "Overlap Cycles " << OverlapCycles << "\n";
            dbgs () << "MinResourceSpan " << MinResourceSpan << "\n";
            report_fatal_error ("Overlap > 1.0");
          }
          
        }else{
          ResourceWithNoInstructions = false;
          
        }
        
        result.clear ();
        selected.clear ();
      }
      while (prev_permutation (selector.begin (), selector.end ()));
    }
    
    
    for (size_t j = 0; j < AverageOverlapsCycles.size(); j++) {
      if (OverlapsCount[j]!=0) {
        AverageOverlapsCycles[j] = AverageOverlapsCycles[j]/OverlapsCount[j];
        AverageOverlaps[j] = AverageOverlaps[j]/OverlapsCount[j];
        
      }
    }
    
    unsigned int MaxOverlapPosition = 0;
    double MaxOverlapCycles = AverageOverlapsCycles[0];
    
    for (size_t j = 1; j < AverageOverlapsCycles.size(); j++) {
      if (AverageOverlapsCycles[j] > MaxOverlapCycles) {
        MaxOverlapCycles = AverageOverlapsCycles[j];
        MaxOverlapPosition = j;
      }
    }
    
    dbgs () << "Resource with max overlap cycles: " << GetResourceName(MaxOverlapPosition) << "\n";
    dbgs () << "MaxOverlapCycles: " << MaxOverlapCycles << "\n";
    
    MaxOverlapPosition = 0;
    double MaxOverlap = AverageOverlaps[0];
    
    for (size_t j = 1; j < AverageOverlaps.size(); j++) {
      if (AverageOverlaps[j] > MaxOverlap) {
        MaxOverlap = AverageOverlaps[j];
        MaxOverlapPosition = j;
      }
    }
    
    dbgs () << "Resource with max overlap percentage: " << GetResourceName(MaxOverlapPosition) << "\n";
    dbgs () << "MaxOverlap: " << MaxOverlap << "\n";
    
    
    vector < int >nonEmptyExecutionUnits;
    for (unsigned j = 0; j < nExecutionUnits; j++) {
      if (InstructionsCountExtended[j] > 0)
        nonEmptyExecutionUnits.push_back (j);
    }
    for (unsigned j = RS_STALL; j <= LFB_STALL; j++) {
      OverlapCycles = 0;
      
      OverlapPercetage = 0;
      if (InstructionsCountExtended[j] > 0) {
        // For every other resource
        vector < int >nonEmptyStalls;
        for (unsigned i = RS_STALL; i <= LFB_STALL; i++) {
          if (i != j && InstructionsCountExtended[i] > 0) {
            nonEmptyStalls.push_back (i);
          }
        }
        // Copy resources into stalls:
        nonEmptyStalls.insert (nonEmptyStalls.end (), nonEmptyExecutionUnits.begin (), nonEmptyExecutionUnits.end ());
        nonEmptyStalls.insert (nonEmptyStalls.begin (), j);
        OverlapCycles = GetOneToAllOverlapCyclesFinal (nonEmptyStalls);
        OverlapPercetage = (float) OverlapCycles / (float (ResourcesSpan[j]));
      }
      dbgs () << j << " " << OverlapCycles;
      fprintf (stderr, " %1.3f\n", OverlapPercetage);
      if (OverlapPercetage > 1.0) {
        dbgs () << "Overlap Cycles " << OverlapCycles << "\n";
        dbgs () << "MinResourceSpan " << MinResourceSpan << "\n";
        report_fatal_error ("Overlap > 1.0");
      }
    }
  }

/*
  printHeaderStat ("Overlaps Derivatives");
  for (unsigned i = 0; i < nExecutionUnits+nBuffers; i++) {
    if (InstructionsCountExtended[i]!= 0) {
      dbgs () << GetResourceName(i) << " " << OverlapsDerivatives[i] << "\n";
    }else{
      if(OverlapsDerivatives[i] != 1)
        report_fatal_error("Derivative not NULL for a resource without instructions");
      else
        dbgs () << GetResourceName(i) << " " << 1 << "\n";
    }
  }
  
  
  // =================== NEW METRIC/HEURISTIC ======================================//
  
  {
    printHeaderStat ("Overlaps metrics");
    for (unsigned i = 0; i < nExecutionUnits+nBuffers; i++) {
      if (InstructionsCountExtended[i]!= 0 && ResourcesSpan[i]!=0) {
        dbgs () << GetResourceName(i) << " " << ((OverlapsMetrics[i]/ResourcesSpan[i])*(TotalSpan/ResourcesSpan[i])) << "\n";
      }else
        dbgs () << GetResourceName(i) << " 0\n";
    }
    
  }
  */
  
  
  {
    
    
    // + 4 to include L1load toghether with L1store, as if they were one resource for L1 cache, L2, L3, and the accesses to mem
    // Before +1 to include L1_load and L1_store. Now +2 because the one before the last in the register file size
    vector < vector < uint64_t > >ResourcesOverlapCycles (nExecutionUnits+nBuffers+2, vector < uint64_t > (nExecutionUnits+nBuffers));
    vector < vector < uint64_t > >ResourcesOnlyLatencyOverlapCycles (nExecutionUnits, vector < uint64_t > (nExecutionUnits+nBuffers));
    vector < vector < uint64_t > >ResourcesOnlyIssueOverlapCycles (nExecutionUnits, vector < uint64_t > (nExecutionUnits+nBuffers));
    bool L1ResourceFound = false;
    for (unsigned i = 0; i < TotalSpan; i++){
      
      vector < unsigned > resourcesInCycle;
      L1ResourceFound = false;
      for (unsigned j = 0; j< nExecutionUnits+nBuffers; j++){
        if (InstructionsCountExtended[j]!= 0 &&  ExecutionUnitsLatency[j]!= 0 &&  CGSFCache[j][i]==1 )
          resourcesInCycle.push_back(j);
      }
      
      for (unsigned j = 0; j< resourcesInCycle.size(); j++){
        ResourcesOverlapCycles[resourcesInCycle[j]][resourcesInCycle.size()]++;
        // l1 calculation
        
        if ((resourcesInCycle[j]==L1_LOAD_CHANNEL || resourcesInCycle[j] == L1_STORE_CHANNEL)){
          if(L1ResourceFound==false){
            L1ResourceFound = true;
            ResourcesOverlapCycles.back()[resourcesInCycle.size()]++;
          }else{
            ResourcesOverlapCycles.back()[resourcesInCycle.size()]--;
            ResourcesOverlapCycles.back()[resourcesInCycle.size()-1]++;
          }
        }
        // End of l1 calculation
        
      }
      
      for (unsigned j = 0; j< nExecutionUnits; j++){
        resourcesInCycle.clear();
        if (InstructionsCountExtended[j]!= 0 &&  ExecutionUnitsLatency[j]!= 0 &&  CISFCache[j][i]==1 ){
          resourcesInCycle.push_back(j);
          for (unsigned k = 0; k< nExecutionUnits; k++){
            if ( k != j && InstructionsCountExtended[k]!= 0 &&   ExecutionUnitsLatency[k]!= 0 && CGSFCache[k][i]==1){
              resourcesInCycle.push_back(k);
            }
          }
        }
        ResourcesOnlyIssueOverlapCycles[j][resourcesInCycle.size()]++;
        
      }
      
      
      
      for (unsigned j = 0; j< nExecutionUnits; j++){
        resourcesInCycle.clear();
        if (InstructionsCountExtended[j]!= 0  &&  ExecutionUnitsLatency[j]!= 0 &&  CLSFCache[j][i]==1){
          resourcesInCycle.push_back(j);
          for (unsigned k = 0; k< nExecutionUnits; k++){
            if ( k != j && InstructionsCountExtended[k]!= 0  &&  ExecutionUnitsLatency[k]!= 0 &&  CGSFCache[k][i]==1){
              resourcesInCycle.push_back(k);
            }
          }
        }
        ResourcesOnlyLatencyOverlapCycles[j][resourcesInCycle.size()]++;
        
      }
      
      
    }
    
    // Add data for caches
    ResourcesOverlapCycles[nExecutionUnits+nBuffers]=ResourcesOverlapCycles[REGISTER_LOAD_CHANNEL];
    ResourcesOverlapCycles.push_back(ResourcesOverlapCycles[L2_LOAD_CHANNEL]);
    ResourcesOverlapCycles.push_back(ResourcesOverlapCycles[L3_LOAD_CHANNEL]);
    ResourcesOverlapCycles.push_back(ResourcesOverlapCycles[MEM_LOAD_CHANNEL]);
    /*
    printHeaderStat ("Ranking ");
    
    
    vector<unsigned> CandidateResources;
    vector< uint64_t  > OverlapCyclesRanking;
    vector<unsigned > TmpElementsToRemove;
    vector<unsigned>::iterator it;
    for (unsigned j = 0; j< nExecutionUnits+nBuffers; j++){
      CandidateResources.push_back(j);
    }
    for (unsigned j = 0; j< nExecutionUnits+nBuffers; j++){
      
      vector<unsigned > ElementsToRemove;
      for (unsigned i = 0; i< nExecutionUnits+nBuffers; i++){
        it = find(CandidateResources.begin(), CandidateResources.end(), i) ;
        if(ResourcesOverlapCycles[i][j]!= 0 && it != CandidateResources.end()){
          //	dbgs() << " "<< ResourcesOverlapCycles[j][i] ;
          
          CandidateResources.erase(it);
          
          if(ElementsToRemove.size() == 0){
            TmpElementsToRemove.clear();
            OverlapCyclesRanking.clear();
          }
          ElementsToRemove.push_back(i);
          TmpElementsToRemove.push_back(i);
          OverlapCyclesRanking.push_back(ResourcesOverlapCycles[i][j]);
          
        }
        
      }
    }
    for (unsigned j = 0; j< nExecutionUnits+nBuffers; j++){
      dbgs () << GetResourceName(j) << " " ;
      uint64_t CyclesOverlapRanking = 0;
      for (unsigned i = 0; i < TmpElementsToRemove.size(); i++){
        if(TmpElementsToRemove[i] == j){
          CyclesOverlapRanking= OverlapCyclesRanking[i];
          break;
        }else{
          CyclesOverlapRanking = 0;
          
        }
      }
      dbgs() << CyclesOverlapRanking<<  "\n";
    }
*/
    printHeaderStat ("Breakdown Overlap");
    
    for (unsigned j = 0; j< nExecutionUnits+nBuffers+5; j++){
      if (j < nExecutionUnits+nBuffers)
        dbgs () << GetResourceName(j);
      else{
        
        if (j == nExecutionUnits+nBuffers)
          dbgs () << "REGISTER";
        
        if (j ==  nExecutionUnits+nBuffers+1)
          dbgs () << "ALL_L1";
        
        if (j == nExecutionUnits+nBuffers+2)
          dbgs () << "L2";
        if (j == nExecutionUnits+nBuffers+3)
          dbgs () << "LLC";
        if (j == nExecutionUnits+nBuffers+4)
          dbgs () << "MEM";
      }
      
      for (unsigned i = 1; i< nExecutionUnits+nBuffers; i++){
        dbgs() << " "<< ResourcesOverlapCycles[j][i] ;
      }
      dbgs() << "\n";
    }
    
    
    printHeaderStat ("Breakdown Overlap - Issue and only latency separated");
    
    
    for (unsigned j = 0; j< nExecutionUnits; j++){
      dbgs () << GetResourceName(j) << " ISSUE";
      for (unsigned i = 1; i< nExecutionUnits+nBuffers; i++){
        dbgs() << " "<< ResourcesOnlyIssueOverlapCycles[j][i] ;
      }
      dbgs() << "\n";
      dbgs () << GetResourceName(j) << " ONLY_LAT";
      for (unsigned i = 1; i< nExecutionUnits+nBuffers; i++){
        dbgs() << " "<< ResourcesOnlyLatencyOverlapCycles[j][i] ;
      }
      dbgs() << "\n";
    }
    
    
    for (unsigned j = nExecutionUnits; j< nExecutionUnits+nBuffers+5; j++){
      if (j < nExecutionUnits+nBuffers)
        dbgs () << GetResourceName(j);
      else{
        
        if (j == nExecutionUnits+nBuffers)
          dbgs () << "REGISTER";
        
        if (j == nExecutionUnits+nBuffers+1)
          dbgs () << "ALL_L1";
        
        if (j == nExecutionUnits+nBuffers+2)
          dbgs () << "L2";
        if (j == nExecutionUnits+nBuffers+3)
          dbgs () << "LLC";
        if (j == nExecutionUnits+nBuffers+4)
          dbgs () << "MEM";
      }
      for (unsigned i = 1; i< nExecutionUnits+nBuffers; i++){
        dbgs() << " "<< ResourcesOverlapCycles[j][i] ;
      }
      dbgs() << "\n";
    }
    
    
    
  }
  
  
  // ===================== ALL OVERLAPS - THIRD APPROACH ===========================//
  {
    
    printHeaderStat ("Overlaps - Each resource with all the others");
    
    vector < int >nonEmptyExecutionUnits;
    
    for (unsigned i = 0; i < nExecutionUnits+nBuffers; i++) {
      OverlapCycles = 0;
      OverlapPercetage = 0;
      nonEmptyExecutionUnits.clear();
      
      if (InstructionsCountExtended[i]!= 0 ) {
        nonEmptyExecutionUnits.push_back(i);
        // For all others
        for (unsigned j = 0; j < nExecutionUnits+nBuffers ; j++) {
          if (j!= i) {
            if (InstructionsCountExtended[j]!= 0) {
              nonEmptyExecutionUnits.push_back(j);
            }
          }
        }
        if(ResourcesSpan[i]!= 0){
          OverlapCycles = GetOneToAllOverlapCyclesFinal (nonEmptyExecutionUnits);
          OverlapPercetage = (float) OverlapCycles / (float (ResourcesSpan[i]));
        }
        dbgs () << GetResourceName(i) << " " << OverlapCycles;
        fprintf (stderr, " %1.3f\n", OverlapPercetage);
        
      }
      
    }
    
  }
  
  
  // ===================== ALL OVERLAPS - ISSUE/LATENCY APPROACH ===========================//
  {
    
    printHeaderStat ("Overlaps - Issue/Latency with all the others");
    
    vector < int >nonEmptyExecutionUnits;
    
    // In this case only execution units
    for (unsigned i = 0; i < nExecutionUnits; i++) {
      OverlapCycles = 0;
      OverlapPercetage = 0;
      nonEmptyExecutionUnits.clear();
      
      if (InstructionsCountExtended[i]!= 0) {
        nonEmptyExecutionUnits.push_back(i);
        for (unsigned j = 0; j < nExecutionUnits+nBuffers ; j++) {
          if (j!= i) {
            if (InstructionsCountExtended[j]!= 0) {
              nonEmptyExecutionUnits.push_back(j);
            }
          }
        }
        if(IssueSpan[i]!=0){
          OverlapCycles = GetOneToAllOverlapCyclesFinal (nonEmptyExecutionUnits, true);
          OverlapPercetage = (float) OverlapCycles / (float (IssueSpan[i]));
        }
        dbgs () << GetResourceName(i) << " ISSUE " << OverlapCycles;
        fprintf (stderr, " %1.3f\n", OverlapPercetage);
        
        
        if (LatencyOnlySpan[i]==0){
          OverlapCycles = 0;
		        OverlapPercetage = 0.0;
        }else{
          OverlapCycles = GetOneToAllOverlapCyclesFinal (nonEmptyExecutionUnits, false); // Latency
          OverlapPercetage = (float) OverlapCycles / (float (LatencyOnlySpan[i]));
        }
        dbgs () << GetResourceName(i) << " ONLY LAT " << OverlapCycles;
        fprintf (stderr, " %1.3f\n", OverlapPercetage);
        
      }
      
    }
    
  }
  
  
  
  
  
  //======================= Bottlenecks ===============================//
  
  printHeaderStat ("Bottlenecks - Buffers stalls added to issue");
  dbgs () << "Bottleneck\tISSUE\tLAT\t";
  for (int j = RS_STALL; j <= LFB_STALL; j++) {
    dbgs () << GetResourceName (j) << "\t";
  }
  dbgs () << "\n";
  uint64_t Work;
  
  for (unsigned i = 0; i < nExecutionUnits; i++) {
    if(InstructionsCountExtended[i] > 0){
      auto & BnkVec = BnkMat[i];
      
      // Work is always the total number of floating point operations... Otherwise it makes
      // no sense to compare with the performance for memory nodes which is calcualted
      // with total work
      Work = InstructionsCount[0];
      dbgs () << GetResourceName (i) << "\t\t";
      
      if (IssueSpan[i] > 0) {
        Performance = (float) Work / ((float) IssueSpan[i]);
        fprintf (stderr, " %1.3f ", Performance);
        
        BnkVec[0] = Performance;
      }
      else {
        dbgs () << INF << "\t";
        
        BnkVec[0] = INF;
      }
      
      if (ResourcesSpan[i] > 0) {
        Performance = (float) Work / ((float) ResourcesSpan[i]);
        fprintf (stderr, " %1.3f ", Performance);
        
        BnkVec[1] = Performance;
      }
      else {
        dbgs () << INF << "\t";
        
        BnkVec[1] = INF;
      }
      
      for (unsigned j = 0; j < nBuffers; j++) {
        if (ResourcesIssueStallSpanVector[i][j] > 0 && ResourcesSpan[j + RS_STALL] != 0) {
          Performance = (float) Work / ((float) ResourcesIssueStallSpanVector[i][j]);
          fprintf (stderr, " %1.3f ", Performance);
          
          BnkVec[j + 2] = Performance;
        }
        else {
          dbgs () << INF << "\t";
          
          BnkVec[j + 2] = INF;
        }
      }
      dbgs () << "\n";
    }else{
      dbgs () << GetResourceName (i) << "\t\t";
      for (unsigned j = 0; j < nBuffers+2; j++) {
        dbgs () << INF << "\t";
      }
      dbgs () << "\n";
    }
  }
  
  //======================= Bottlenecks ===============================//
  {
    printHeaderStat ("Bottlenecks - Buffers stalls added to latency");
    dbgs () << "Bottleneck\tISSUE\tLAT\t";
    for (int j = RS_STALL; j <= LFB_STALL; j++) {
      dbgs () << GetResourceName (j) << "\t";
    }
    dbgs () << "\n";
    uint64_t Work;
    
    for (unsigned i = 0; i < nExecutionUnits; i++) {
      if(InstructionsCountExtended[i] > 0){
        auto & BnkVec = BnkMat[i];
        
        // Work is always the total number of floating point operations... Otherwise it makes
        // no sense to compare with the performance for memory nodes which is calcualted
        // with total work
        Work = InstructionsCount[0];
        dbgs () << GetResourceName (i) << "\t\t";
        
        if (IssueSpan[i] > 0) {
          Performance = (float) Work / ((float) IssueSpan[i]);
          fprintf (stderr, " %1.3f ", Performance);
          
          BnkVec[0] = Performance;
        }
        else {
          dbgs () << INF << "\t";
          
          BnkVec[0] = INF;
        }
        
        if (ResourcesSpan[i] > 0) {
          Performance = (float) Work / ((float) ResourcesSpan[i]);
          fprintf (stderr, " %1.3f ", Performance);
          
          BnkVec[1] = Performance;
        }
        else {
          dbgs () << INF << "\t";
          
          BnkVec[1] = INF;
        }
        
        for (unsigned j = 0; j < nBuffers; j++) {
          if (ResourcesIssueStallSpanVector[i][j] > 0 && ResourcesSpan[j + RS_STALL] != 0) {
            Performance = (float) Work / ((float) ResourcesStallSpanVector[i][j]);
            fprintf (stderr, " %1.3f ", Performance);
            
            BnkVec[j + 2] = Performance;
          }
          else {
            dbgs () << INF << "\t";
            
            BnkVec[j + 2] = INF;
          }
        }
        dbgs () << "\n";
      }else{
        dbgs () << GetResourceName (i) << "\t\t";
        for (unsigned j = 0; j < nBuffers+2; j++) {
          dbgs () << INF << "\t";
        }
        dbgs () << "\n";
      }
    }
  }
  
  
  
  //======================= Buffers Bottlenecks ===============================//
  
  printHeaderStat ("Buffers Bottlenecks");
  {
    uint64_t Work = InstructionsCount[0];
    dbgs () << "Bottleneck\tISSUE\n";
    for (int j = RS_STALL; j <= LFB_STALL; j++) {
      dbgs () << GetResourceName (j) << "\t";
      if(InstructionsCountExtended[j]!=0)
        fprintf (stderr, " %1.3f\n", ((float)Work)/ResourcesSpan[j]);
      else
        dbgs () << INF << "\n";
    }
    
  }
  
  //=============== Botttlenecks without buffers and latency separate =========//
  
  
  {
    printHeaderStat ("Bottlenecks II");
    dbgs () << "Bottleneck\tISSUE\tLAT_ONLY\n";
    
    uint64_t Work;
    
    for (unsigned i = 0; i < nExecutionUnits; i++) {
      if(InstructionsCountExtended[i] > 0){
        // Work is always the total number of floating point operations... Otherwise it makes
        // no sense to compare with the performance for memory nodes which is calcualted
        // with total work
        Work = InstructionsCount[0];
        dbgs () << GetResourceName (i) << "\t\t";
        
        if (IssueSpan[i] > 0) {
          Performance = (float) Work / ((float) IssueSpan[i]);
          fprintf (stderr, " %1.3f ", Performance);
          
        }
        else {
          dbgs () << INF << "\t";
        }
        
        if (LatencyOnlySpan[i] > 0) {
          Performance = (float) Work / ((float) LatencyOnlySpan[i]);
          fprintf (stderr, " %1.3f ", Performance);
        }
        else {
          dbgs () << INF << "\t";
          
        }
        
        dbgs () << "\n";
      }else{
        dbgs () << GetResourceName (i) << "\t\t";
        for (unsigned j = 0; j < 2; j++) {
          dbgs () << INF << "\t";
        }
        dbgs () << "\n";
      }
    }
  }
  
  
  //===================== Issue latency overlap  ==============================//
  
  printHeaderStat ("Issue/Only Latency Cycles");
  
  for(uint i = 0; i< nExecutionUnits; i++){

    dbgs() << GetResourceName(i) << " " <<  IssueSpan[i] << " " << LatencyOnlySpan[i] << "\n";
    
    // }
  }
  
  
  //======================= Execution Times Breakdown =========================//
  printHeaderStat ("Execution Times Breakdowns");
  dbgs () << "RESOURCE\tMIN-EXEC-TIME\tISSUE-EFFECTS\tLATENCY-EFFECTS\tSTALL-EFFECTS\tTOTAL\n";
  
  unsigned MinExecutionTime;
  unsigned IssueEffects;
  unsigned LatencyEffects;
  unsigned StallEffects;
  double Throughput = 0;
  
  for (unsigned i = 0; i < nExecutionUnits; i++) {
    
    
    if (InstructionsCountExtended[i] == 0) {
      MinExecutionTime = 0;
      LatencyEffects = 0;
      IssueEffects = 0;
      StallEffects = ResourcesTotalStallSpanVector[i];
      dbgs () << GetResourceName (i) << "\t\t";
      dbgs () << " " << MinExecutionTime;
      dbgs () << "\t";
      dbgs () << " " << IssueEffects;
      dbgs () << "\t";
      dbgs () << " " << LatencyEffects;
      dbgs () << "\t";
      dbgs () << " " << StallEffects;
      // fprintf(stderr, " %1.3f ", StallEffects);
      if (MinExecutionTime + IssueEffects + LatencyEffects + StallEffects != ResourcesTotalStallSpanVector[i]
          && MinExecutionTime != 0) {
        report_fatal_error ("Breakdown of execution time does not match total execution time\n");
        
      }
      else {
        dbgs () << "\t" << ResourcesTotalStallSpanVector[i] << "\n";
      }
      
    }
    else {
      // TODO: Do not assume is not vector code
      Throughput= GetEffectiveThroughput(i, AccessWidths[i],1);
      

      //  if (ExecutionUnitsLatency[i] == 0 && Throughput == INF) {
      if (ExecutionUnitsLatency[i] == 0) {
        MinExecutionTime = 0;
      }
      else {
        if (i < nArithmeticExecutionUnits + nMovExecutionUnits) {
          if (Throughput == INF) {
            MinExecutionTime = 1;
          }
          else {
            
            MinExecutionTime = (unsigned) ceil (InstructionsCountExtended[i]
                                                / Throughput);
          }
        }
        else {
          if (Throughput == INF) {
            MinExecutionTime = 1;
          }
          else {
            MinExecutionTime = (unsigned)
            ceil (InstructionsCountExtended[i]/ Throughput);
            //  MinExecutionTime = (unsigned)
            // ceil ((InstructionsCountExtended[i] * AccessGranularities[i]) / (Throughput));
            DEBUG (dbgs () << "AccessGranularities[i] " << AccessGranularities[i] << "\n");
            
            DEBUG (dbgs () << "Throughput " << Throughput << "\n");
            DEBUG (dbgs () << "MinExecutionTime " << Throughput << "\n");
          }
        }
      }
      
      if (VectorCode == false) {
        if (IssueSpan[i] != 0 && IssueSpan[i] < MinExecutionTime) {
          
          dbgs () << "IssueSpan[i] " << IssueSpan[i] << "\n";
          dbgs () << "MinExecutionTime " << MinExecutionTime << "\n";
          report_fatal_error ("IssueSpan < Min execution time");
        }
      }
      else {
        
      }
      
      if (ScalarInstructionsCountExtended[i] != 0 && VectorInstructionsCountExtended[i] != 0) {
        
        unsigned AdjustedIssueSpan = IssueSpan[i];
        if (ExecutionUnitsParallelIssue[i] != INF) {
          if (ExecutionUnitsThroughput[i] != INF) {
            
            if (AccessWidths[i] * VectorWidth > (ExecutionUnitsThroughput[i] * ExecutionUnitsParallelIssue[i])) {
              // Issue Span calculated as before needs correction.
              unsigned IssueGranularity =
              ceil (AccessWidths[i] * VectorWidth / (ExecutionUnitsThroughput[i] * ExecutionUnitsParallelIssue[i]));
              
              AdjustedIssueSpan = AdjustedIssueSpan + VectorInstructionsCountExtended[i] * (IssueGranularity - 1);	// Because span already includes the firs issue cycle.
              
            }
          }
          else {		// If Throughout and ExecutionUnitsParallelIssue are INF, don't have to adjust anything from issue span
            
          }
        }
        else {		// If ExecutionUnitsParallelIssue in INF but Thoroughput no
          if (ExecutionUnitsThroughput[i] != INF) {
            if (AccessWidths[i] * VectorWidth > (ExecutionUnitsThroughput[i])) {
              // Issue Span calculated as before needs correction.
              unsigned IssueGranularity = ceil (AccessWidths[i] * VectorWidth / (ExecutionUnitsThroughput[i]));
              
              AdjustedIssueSpan = AdjustedIssueSpan + VectorInstructionsCountExtended[i] * (IssueGranularity - 1);	// Because span already includes the firs issue cycle.
              
            }
          }
          
        }
        IssueEffects = AdjustedIssueSpan - MinExecutionTime;
        if (ResourcesSpan[i] != 0) {
          LatencyEffects = ResourcesSpan[i] - AdjustedIssueSpan;
        }
      }
      else {
        IssueEffects = IssueSpan[i] - MinExecutionTime;
        if (ResourcesSpan[i] != 0) {
          LatencyEffects = ResourcesSpan[i] - IssueSpan[i];
        }
      }
      
      StallEffects = ResourcesTotalStallSpanVector[i] - ResourcesSpan[i];
      
      
      
      dbgs () << GetResourceName (i) << "\t\t";
      dbgs () << " " << MinExecutionTime;
      dbgs () << "\t";
      dbgs () << " " << IssueEffects;
      dbgs () << "\t";
      dbgs () << " " << LatencyEffects;
      dbgs () << "\t";
      dbgs () << " " << StallEffects;
      if (MinExecutionTime + IssueEffects + LatencyEffects + StallEffects != ResourcesTotalStallSpanVector[i]
          && MinExecutionTime != 0) {
        report_fatal_error ("Breakdown of execution time does not match total execution time\n");
        
      }
      else {
        dbgs () << "\t" << ResourcesTotalStallSpanVector[i] << "\n";
      }
    }
  }
  

  
  unsigned nArithmeticInstructionCount =
  InstructionsCountExtended[FP_ADD_NODE] + InstructionsCountExtended[FP_MUL_NODE] + InstructionsCountExtended[FP_DIV_NODE];
  printHeaderStat ("TOTAL");
  //TODO: add division!
  dbgs () << "TOTAL FLOPS" << "\t" << nArithmeticInstructionCount << "\t\t" << CalculateGroupSpanFinal (compResources) << " \n";
  dbgs () << "TOTAL MOV/SHUFFLE/BLEND" << "\t" <<
  InstructionsCountExtended[FP_SHUFFLE_NODE] +
  InstructionsCountExtended[FP_BLEND_NODE] +
  InstructionsCountExtended[FP_MOV_NODE] << "\t\t" << CalculateGroupSpanFinal (movResources) << " \n";
  dbgs () << "TOTAL MOPS" << "\t" << InstructionsCount[1] << "\t\t" << CalculateGroupSpanFinal (memResources) << " \n";
  dbgs () << "TOTAL" << "\t\t" << InstructionsCount[0] + InstructionsCount[1] << "\t\t" << TotalSpan << " \n";
  Performance = (float) nArithmeticInstructionCount / ((float) TotalSpan);
  fprintf (stderr, "PERFORMANCE %1.3f\n", Performance);
  dbgs () << " NRegisterSpills " << "\t" << NRegisterSpills <<" \n";
  
  
  
#ifdef SOURCE_CODE_ANALYSIS
  
  printHeaderStat ("SOURCE CODE LINE INFO");
  
  
  //First, iterate over the map that contains an entry for each code line, and the value
  //mapped is a set of all the distinct cycles to which this source code line contributes to
  typedef unordered_map < uint64_t, set < uint64_t > >::iterator it_type;
  typedef unordered_map < uint64_t, vector < uint64_t > >::iterator it_type2;
  
  
  // Get all source code lines:
  vector < unsigned >AllSourceCodeLines;
  for (it_type iterator = SourceCodeLineInfo.begin (); iterator != SourceCodeLineInfo.end (); iterator++) {
    AllSourceCodeLines.push_back (iterator->first);
  }
  std::cout << "\n";
  // Sort the vector
  std::sort (AllSourceCodeLines.begin (), AllSourceCodeLines.end ());
  
  std::map < unsigned, unsigned >AdjustedSourceCodeLines;
  for (std::vector < unsigned >::iterator it = AllSourceCodeLines.begin (); it != AllSourceCodeLines.end () - 1; ++it) {
    AdjustedSourceCodeLines[*it] = *(it + 1);
  }
  /*
   for(auto it = AdjustedSourceCodeLines.cbegin(); it != AdjustedSourceCodeLines.cend(); ++it)
   {
   std::cout << it->first << " " << it->second << "\n";
   }
   */
  for (it_type iterator = SourceCodeLineInfo.begin (); iterator != SourceCodeLineInfo.end (); iterator++) {
    dbgs () << "Source Code Line " << AdjustedSourceCodeLines[iterator->first] << "\n";
    for (it_type iteratorSourceCodeLineOperations =
         SourceCodeLineOperations.begin (); iteratorSourceCodeLineOperations != SourceCodeLineOperations.end ();
         iteratorSourceCodeLineOperations++) {
      if (iteratorSourceCodeLineOperations->first == iterator->first) {
        dbgs () << "Operations in line :";
        for (std::set < uint64_t >::iterator it = iteratorSourceCodeLineOperations->second.begin ();
             it != iteratorSourceCodeLineOperations->second.end (); ++it) {
          dbgs () << " " << GetResourceName (*it);
        }
        dbgs () << "\n";
        
      }
      
    }
    
    unordered_map < uint64_t, vector < uint64_t > >::iterator it = SourceCodeLineInfoBreakdown.find (iterator->first);
    if (it == SourceCodeLineInfoBreakdown.end ())
      report_fatal_error ("Source code line not found\n");
    else {
      // Iterate over the vector
      for (unsigned i = 0; i < it->second.size (); i++) {
        dbgs () << " " << it->second[i];
      }
      dbgs () << "\n";
    }
    dbgs () << "__________________________________________________________\n";
  }
  
#endif
  
  
}

