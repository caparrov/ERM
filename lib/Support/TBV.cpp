
<<<<<<< HEAD
#define INTERPRETER
=======
//#define INTERPRETER
>>>>>>> 0f5a3bded113e34dc5ca66986bd8d881911da8b6

#ifdef INTERPRETER
#include "llvm/Support/DynamicAnalysis.h"
#else
#include "DynamicAnalysis.h"
#endif


TBV::TBV()
{
    tbv_map.resize(SplitTreeRange);
    e = true;
}


#ifdef EFF_TBV
bool  TBV_node::empty()
{
return e;
    //return (BitVector.size()==SplitTreeRange && BitVector.count()==0);
}


#endif
bool TBV::empty()
{
    return e;
}


bool  TBV::get_size(){
return tbv_map.size();
}

void TBV::resize(){
 //tbv_map.resize(new_size);
tbv_map.push_back(TBV_node());
}



#ifdef EFF_TBV

void  TBV_node::insert_node( uint64_t bitPosition)
{
e = true;
   BitVector[bitPosition] = 1;

}


#endif


void TBV::insert_node(uint64_t key, unsigned bitPosition)
{
    key = key % SplitTreeRange;
    e = false;
    tbv_map[key].BitVector[bitPosition] = 1;

}




#ifdef SOURCE_CODE_ANALYSIS
void TBV::insert_source_code_line(uint64_t key, unsigned SourceCodeLine, unsigned Resource)
{
    key = key % SplitTreeRange;
    tbv_map[key].SourceCodeLinesOperationPair.push_back(std::make_pair(SourceCodeLine,Resource));
}

vector<pair<unsigned,unsigned>> TBV::get_source_code_lines(uint64_t key){
 key = key % SplitTreeRange;
  return tbv_map[key].SourceCodeLinesOperationPair;
}
#endif


void TBV::delete_node(uint64_t key, unsigned bitPosition)
{
    key = key % SplitTreeRange;
    tbv_map[key].BitVector[bitPosition] = 0;
}


#ifdef EFF_TBV

bool  TBV_node::get_node(uint64_t bitPosition)
{
  if (empty()) return false;

  return BitVector[bitPosition] == 1;

}
#endif


bool TBV::get_node(uint64_t key, unsigned bitPosition)
{
  if (empty()) return false;
  
    key = key % SplitTreeRange;
//key = key & (SplitTreeRange-1);
	//key = key - SplitTreeRange;
    return (tbv_map[key].BitVector[bitPosition] == 1);

}




#ifdef EFF_TBV

bool  TBV_node::get_node_nb(uint64_t bitPosition)
{
  if (empty()) return false;

  return BitVector[bitPosition] == 0;

}
#else
bool TBV::get_node_nb(uint64_t key, unsigned bitPosition)
{
    if (empty()) return false;
    key = key % SplitTreeRange;
    return (tbv_map[key].BitVector[bitPosition] == 0);
}
#endif
uint64_t BitScan(vector< TBV> &FullOccupancyCyclesTree, uint64_t key, unsigned bitPosition)
{
    uint64_t kLocal = key % SplitTreeRange;
    uint64_t chunk = kLocal / SplitTreeRange;
    
    while (chunk < FullOccupancyCyclesTree.size())
    {
        while (kLocal < SplitTreeRange)
        {
            if (FullOccupancyCyclesTree[chunk].get_node(kLocal, bitPosition)) return (kLocal + chunk * SplitTreeRange);
            kLocal++;
        }
        kLocal = 0;
        chunk++;
    }
    
    return key;
}
