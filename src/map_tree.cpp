#include "robmob_pkg/map_tree.hpp"
	
	
MapTree::MapTree(Map const& map)
	: _map(map), _distRef( std::sqrt(_map.getHeight()*_map.getHeight() + _map.getWidth()*_map.getWidth()) / CONNEXITY_RATE)
{}
	
void MapTree::generateRandVertices()
{
	std::srand(std::time(nullptr));	

	for(int i(0); i< NB_PTS_MAX; ++i)
	{
		Pos p(std::rand()%_map.getHeight(), std::rand()%_map.getWidth());
		_vertices.push_back(p);
	}

} 


std::vector<Pos> MapTree::computeShorestPath(const Pos& src, const Pos& dest)
{
	generateRandVertices(); // rand positions all over the map
	
	selectVertices(); // node selection : delete wrong pos, compute mean pos for gathered points
	
	initGraph(src, dest); // build the graph by adding source and destination points
	
	dijkstra(); // compute dijkstra algorithm to find shortest path
	
	return getShortestPath(); // return shorest path
}



void MapTree::selectVertices()
{
	std::vector<Pos> selectedVertices;
	
	
	for (size_t ind(0) ; ind < _vertices.size() ; ++ind)
	{
		if(_map.getPxlValBinEroded(_vertices[ind].i, _vertices[ind].j) == 255) //if the pos is available  
		{	
			bool closePointFound(false);
			
			for(size_t indSlct(0) ; indSlct < selectedVertices.size() ; ++indSlct)
			{
				if( isNear(_vertices[ind], selectedVertices[indSlct], _distRef /10 ) )
				{
					closePointFound = true;
					
					// Keep an average point
					selectedVertices[indSlct].i = (selectedVertices[indSlct].i + _vertices[ind].i) / 2 ;
					selectedVertices[indSlct].j = (selectedVertices[indSlct].j + _vertices[ind].j) / 2 ;
					break;
				}
			}
			
			if( !closePointFound ) // if too close point has not been found
				selectedVertices.push_back(_vertices[ind]);
		} 	
	}
	
	_vertices = selectedVertices;
}


bool MapTree::isNear(const Pos& p1, const Pos& p2, const float& distRef )
{
	return ( std::sqrt((p1.i - p2.i)*(p1.i - p2.i) + (p1.j - p2.j)*(p1.j - p2.j)) < distRef );
}



void MapTree::dispVerticesInfos()
{
	for(auto&& v : _vertices)
		v.disp();
		
}


void MapTree::initGraph(const Pos& source, const Pos& target)
{

	// Add the source and the target to the vertices array 
	_vertices.push_back(source); 
	_vertices.push_back(target);

    // Graph init
    G = new std::vector<std::pair<int,int>>[_vertices.size()]; 
    
    for(size_t i(0); i < _vertices.size(); ++i)
    {
    	for(size_t j(0); j < _vertices.size(); ++j)
    	{
    		if (isNear(_vertices[i], _vertices[j], _distRef) ) // neighbor
    		{
    			if(isWayClear(_vertices[i], _vertices[j]) ) // no obstacle
    			{
		 			int w = static_cast<int>(std::sqrt(
		 							(_vertices[i].i - _vertices[j].i)*(_vertices[i].i - _vertices[j].i) + 
		 							(_vertices[i].j - _vertices[j].j)*(_vertices[i].j - _vertices[j].j)
		 															));
		 			G[i].push_back({j,w});
		         G[j].push_back({i,w});
		      }
    		}
    	}
    }

}


bool MapTree::isWayClear(const Pos& p1, const Pos& p2)
{
	// sampling
	float di(0), dj(0);
	float sampling(std::sqrt((p1.i - p2.i)*(p1.i - p2.i) + (p1.j - p2.j)*(p1.j - p2.j)));

	for(int it(0) ; it < sampling ; ++it)
	{
		if( _map.getPxlValBinEroded( p1.i + di, p1.j + dj) == 0 ) // if there is an obstacle
			return false; 
		
		di += (p2.i - p1.i)*1.0 / sampling;
		dj += (p2.j - p1.j)*1.0 / sampling ; 
	}
	return true;
}





void MapTree::dijkstra() 
{
	int src = _vertices.size()-2;
	
	D.assign(_vertices.size(), INF);
	D[src] = 0;
	
	std::set<std::pair<int,int>> Q;
	Q.insert({0,src});
	
	while(!Q.empty()) 
	{
		auto top = Q.begin();
		int u = top->second;
		Q.erase(top);
		
		for(auto next: G[u])    
		{
			int v = next.first, weight = next.second;
			
			if( D[v] > D[u] + weight) 
			{
				if(Q.find( {D[v], v} ) != Q.end())
					Q.erase(Q.find( {D[v], v} ));
					
				D[v] = D[u] + weight;
				Q.insert( {D[v], v} );
			}
		}
	}
	/*
	for(size_t i=0;i<_vertices.size();i++)
            std::cout<<"D[" << i << "] = " << D[i] << " "  ;
        std::cout<< std::endl; 
	*/
}


std::vector<Pos> MapTree::getShortestPath()
{
	std::vector<Pos> res;
	
	int ind = _vertices.size()-1; // start from the target to reach
	res.push_back(_vertices[ind]); // add its pos to res
	
	while(D[ind] != 0) // until to the starting pos
	{
		int optInd(0);
		
		for(size_t indPrev(0) ; indPrev < G[ind].size(); ++indPrev)
		{
			int w = G[ind][indPrev].second;
			int indTest = G[ind][indPrev].first;
			if(w != 0) // avoid itself
			{
				if(D[ind] - w == D[indTest]) // get index corresponding to the optimal previous node for shortest path
				{
					optInd = indTest;
					break;
				}
			}	
		}
		
		res.push_back(_vertices[optInd]); // add it to res
		ind = optInd; // iterate
		
	}
	
	// reverse array : start : 0 -> end : size()-1
	std::vector<Pos> reversedRes;
	for(size_t ind(0) ; ind < res.size() ; ++ind)
		reversedRes.push_back(res[res.size() - 1 - ind]);
	
	
	
	return reversedRes;
}




