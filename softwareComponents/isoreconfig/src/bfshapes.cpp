#include <cassert>
#include <queue>

#include <isoreconfig/isomorphic.hpp>
#include <isoreconfig/bfshapes.hpp>

using VisitedId = size_t;

namespace rofi::isoreconfig {

using namespace rofi::configuration;
using Predecessors = std::unordered_map< VisitedId, VisitedId >;

void saveToFile( const RofiWorld& bot, const std::string& path )
{
    auto jason = serialization::toJSON( bot );
    std::ofstream out(path);
    // std::cout << jason.dump() << "\n";
    out << jason.dump();
    out.close();
}

bool withinBounds( const std::span< float >& values, 
    const std::span< const std::pair< float, float > >& bounds )
{
    auto valIt = values.begin();
    for ( auto[low, high] : bounds )
    {
        if ( *valIt < low || high < *valIt ) 
            return false;
        ++valIt;
    }

    return true;
}

bool operator==( const Joint& j1, const Joint& j2 )
{
    return std::equal( 
            j1.jointLimits().begin(), j1.jointLimits().end(), 
            j2.jointLimits().begin() )
        && std::equal( 
            j1.positions().begin(), j1.positions().end(), 
            j2.positions().begin() );
}

bool operator==( const RoficomJoint& rj1, const RoficomJoint& rj2 )
{
    return rj1.orientation == rj2.orientation
        && rj1.sourceModule == rj2.sourceModule
        && rj1.destModule == rj2.destModule
        && rj1.sourceConnector == rj2.sourceConnector
        && rj1.destConnector == rj2.destConnector
        && std::equal( 
            rj1.jointLimits().begin(), rj1.jointLimits().end(), 
            rj2.jointLimits().begin() )
        && std::equal( 
            rj1.positions().begin(), rj1.positions().end(), 
            rj2.positions().begin() );
}

bool operator==( const SpaceJoint& sj1, const SpaceJoint& sj2 )
{
    return *sj1.joint == *sj2.joint
        // && sj1.refPoint == sj2.refPoint // does not work, is it neccessary?
        && sj1.destModule == sj2.destModule 
        && sj1.destComponent == sj2.destComponent;
}

bool operator==( const ComponentJoint& cj1, const ComponentJoint& cj2 )
{
    return *cj1.joint == *cj2.joint
        && cj1.sourceComponent == cj2.sourceComponent
        && cj1.destinationComponent == cj2.destinationComponent;
}

bool operator==( const Module& mod1, const Module& mod2 )
{
    return mod1.type == mod2.type
        && mod1.getId() == mod2.getId()
        // if modules have same type, we do not care about components?
        // && std::equal( mod1.components().begin(), mod1.components().end(), mod2.components().begin() )
        && std::equal( 
            mod1.joints().begin(), mod1.joints().end(), 
            mod2.joints().begin(),
            []( const auto& cj1, const auto& cj2 ){ return cj1 == cj2; } );    
}

bool operator==( const RofiWorld::ModuleInfo& modInf1, const RofiWorld::ModuleInfo& modInf2 )
{
    // ignores std::vector< SpaceJointHandle > spaceJoints;
    return *modInf1.module == *modInf2.module;
}

bool operator==( const RofiWorld& rw1, const RofiWorld& rw2 )
{
    // ignores atoms::HandleSet< SpaceJoint > _spaceJoints;
    if ( rw1.modules().size() != rw2.modules().size() 
        || rw1.roficomConnections().size() != rw2.roficomConnections().size()
        || rw1.referencePoints().size() != rw2.referencePoints().size() )
        return false;

    return std::equal( 
            rw1.modules().begin(), rw1.modules().end(), 
            rw2.modules().begin(),
            []( const auto& mod1, const auto& mod2 ){ return mod1 == mod2; } )
        && std::equal( 
            rw1.roficomConnections().begin(), rw1.roficomConnections().end(), 
            rw2.roficomConnections().begin(),
            []( const auto& rj1, const auto& rj2 ){ return rj1 == rj2; } );
}

bool equalConfig( const RofiWorld& rw1, const RofiWorld& rw2 )
{
    // return equalShape( rw1, rw2 ); // Shape equality
    return rw1 == rw2; // Configuration equality
}

void generateParametersRec( std::vector< std::vector< float > >& result, std::vector< float >& current,
     const std::array< float, 3 >& possChange, size_t toBeAdded )
{
    if ( toBeAdded == 0 ) 
    {
        result.push_back( current );
        return;
    }

    for ( float change : possChange )
    {
        current.push_back( change );
        generateParametersRec( result, current, possChange, toBeAdded - 1 );
        assert( current.size() > 0 );
        current.pop_back();
    }
}

std::vector< std::vector< float > > generateParameters( size_t dog, float step ) 
{
    std::vector< std::vector< float > > result;
    std::array< float, 3 > possChange { -step, step, 0 };
    std::vector< float > current;

    generateParametersRec( result, current, possChange, dog );
    assert( result.back() == std::vector< float >( dog, 0 ) );
    result.pop_back(); // Remove identity
    return result;
}

// Get possible configurations made from the current one
// "1 step" away, (TODO ignoring configurations of identical classes?)
std::vector< RofiWorld > getDescendants(
    const RofiWorld& current, float step ) 
{
    std::vector< RofiWorld > result;

    // Descendants generated by changing joint parameters (e. g. rotation of a module)
    for ( const auto& modInf : current.modules() )
    {
        for ( size_t j = 0; j < modInf.module->joints().size(); ++j )
        {
            auto& currJoint = modInf.module->joints()[j].joint;
            // TODO generate rotations for all reocurring dogs in advance
            for ( auto& possRot : generateParameters( currJoint->positions().size(), step ) )
            {                
                RofiWorld newBot = current;

                // Shouldnt work - is const
                // newBot.getModule(  modInf.module->getId() )->joints()[j].joint->changePositions( possRot );

                // Skip rotation if it does not respect joint bounds
                if ( !newBot.getModule(  modInf.module->getId() )->changeJointPositionsBy( int(j), possRot ).has_value() )
                    continue;

                if ( newBot.prepare().has_value() && newBot.isValid() )
                    result.push_back( newBot );
            }
        }
    }

    // Descendants generated by disconnecting already connected roficoms
    auto allConnects = current.roficomConnections();
    for ( auto start = allConnects.begin(); start != allConnects.end(); ++start )
    {
        RofiWorld newBot = current;
        newBot.disconnect( start.get_handle() );

        if ( newBot.prepare().has_value() && newBot.isValid() )
            result.push_back( newBot );
    }

    // Generate all possible new connections from mod-comp to mod-comp
    std::vector< std::tuple< ModuleId, int, ModuleId, int, roficom::Orientation > > possConnect;
    for ( const auto& modInf : current.modules() )
    {
        assert( modInf.module->type == ModuleType::Universal );

        for ( const auto& conn : modInf.module->connectors() )
        {
            assert( conn.type == ComponentType::Roficom );
            std::optional<std::pair< const Component&, roficom::Orientation >> poss = conn.getNearConnector();

            if ( !poss ) continue; // No adjacent roficom to <conn>

            assert( poss->first.type == ComponentType::Roficom );
            assert( poss->first.parent );

            ModuleId mod1 = modInf.module->getId();
            int comp1 = conn.getIndexInParent();
            ModuleId mod2 = poss->first.parent->getId();
            int comp2 = poss->first.getIndexInParent();

            bool alreadyConnected = false;
            for ( const auto& rofiJoint : current.roficomConnections() )
            {
                int sourceMod = rofiJoint.getSourceModule( current ).getId();
                int destMod = rofiJoint.getDestModule( current ).getId();
                if ( ( sourceMod == mod1 && rofiJoint.sourceConnector == comp1 
                    && destMod == mod2 && rofiJoint.destConnector == comp2 ) ||
                    ( sourceMod == mod2 && rofiJoint.sourceConnector == comp2 
                    && destMod == mod1 && rofiJoint.destConnector == comp1 ) )
                { 
                    alreadyConnected = true; 
                    break; 
                }
            }
            if ( alreadyConnected ) 
                continue;

            auto newConn = std::tie( mod1, comp1, mod2, comp2, poss->second );
            possConnect.push_back( newConn );
        }
    }

    // filter duplicates
    std::vector< std::tuple< ModuleId, int, ModuleId, int, roficom::Orientation > > noDupes;
    for ( const auto&[ conn1mod1, conn1comp1, conn1mod2, conn1comp2, conn1ori ] : possConnect )
    {
        // Connector cannot be connected to itself
        assert( conn1mod1 != conn1mod2 || conn1comp1 != conn1comp2 );

        bool add = true;
        for ( const auto&[ conn2mod1, conn2comp1, conn2mod2, conn2comp2, _ ] : noDupes )
        {
            // Every connector can be connected only to one other connector
            assert( conn1mod1 != conn2mod1 || conn1comp1 != conn2comp1 );
            
            if ( conn1mod1 == conn2mod2 && conn1comp1 == conn2comp2 )
            { 
                add = false; 
                break; 
            }
        }
        if ( add ) 
            noDupes.push_back( std::tie( conn1mod1, conn1comp1, conn1mod2, conn1comp2, conn1ori ) );
    }
    possConnect = std::move( noDupes );

    // Generate new rofibots by connecting adjacent roficoms
    for ( const auto&[ mod1, comp1, mod2, comp2, orient ] : possConnect )
    {
        RofiWorld newBot = current;
        connect( 
            newBot.getModule( mod1 )->components()[ comp1 ], 
            newBot.getModule( mod2 )->components()[ comp2 ],
            orient );
        assert( current.isPrepared() );
        
        if ( newBot.prepare().has_value() && newBot.isValid() )
            result.push_back( newBot );
    }

    return result;
}

// Make a vector of predecessors of target configuration from map of predecessors
// Assume target is reachable from start in predecessors map
std::vector< RofiWorld > getPredecessors( const Shapes& visited,
    Predecessors& predecessor, 
    VisitedId target, VisitedId start )
{
    assert( predecessor.contains( start ) );
    assert( predecessor.contains( target ) );

    std::vector<RofiWorld> result;
    VisitedId current = target;
    
    while ( current != start ) {
        result.push_back( visited[current] );
        // current must always have a predecessor or given map is not correct
        assert( predecessor.contains( current ) );
        current = predecessor.find( current )->second;
    }

    result.push_back( visited[start] );
    std::reverse( result.begin(), result.end() );
    return result;
} 

// Assume start and target consist only of **universal** modules
std::vector<RofiWorld> bfsShapes(
    const RofiWorld& start, const RofiWorld& target,
    float step, Reporter& rep )
{
    Shapes visited;
    Predecessors predecessor;
    std::unordered_map< VisitedId, size_t > distance;
    size_t layer = 0;

    VisitedId startId = visited.insert( start ); rep.onUpdateVisited( visited );
    // Starting configuration has itself as predecessor and distance of 0
    predecessor.insert( { startId, startId } ); rep.onUpdatePredecessors( predecessor );
    distance.insert( { startId, layer } ); rep.onUpdateDistance( distance );
    rep.onUpdateLayer( layer );
    // std::cout << rep.toString() << "\n";

    rep.onNewDescendant( start );
    if ( equalConfig( start, target ) ) 
    {
        rep.onReturn( true );
        return getPredecessors( visited, predecessor, startId, startId );
    }

    std::queue< VisitedId > bfsQueue;
    bfsQueue.push( startId ); rep.onUpdateQueue( bfsQueue );

    while ( !bfsQueue.empty() ) 
    {
        VisitedId current = bfsQueue.front();
        bfsQueue.pop(); rep.onUpdateQueue( bfsQueue );

        // We are adding to a different layer
        if ( distance.find( current )->second != layer )
        {
            assert( ( distance.find( current )->second == 0 && current == startId ) || 
                layer == distance.find( current )->second - 1 );
            ++layer; rep.onUpdateLayer( layer );
            std::cout << rep.toString() << "\n";
        }

        std::vector< RofiWorld > descendants = getDescendants( visited[current], step );
        rep.onGenerateDescendants( descendants );

        for ( const RofiWorld& child : descendants ) {
            rep.onNewDescendant( child );
            if ( visited.contains( child ) ) 
                continue;

            VisitedId childId = visited.insert( child ); rep.onUpdateVisited( visited );
            predecessor.insert({ childId, current }); rep.onUpdatePredecessors( predecessor );
            assert( distance.find( current ) != distance.end() ); // current must have been assigned distance already
            distance.insert({ childId, distance.find( current )->second + 1 }); rep.onUpdateDistance( distance );

            if ( equalConfig( child, target ) ) 
            {
                rep.onReturn( true );
                return getPredecessors( visited, predecessor, childId, startId );
            }

            bfsQueue.push( childId ); rep.onUpdateQueue( bfsQueue );
        }
    }

    // Target is not reachable from start using step rotations and dis/connections
    rep.onReturn( false );
    return {};
}

} // namespace rofi::isoreconfig
