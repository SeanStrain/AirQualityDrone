package uk.ac.ed.inf.aqmaps;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

/**
 * The Drone class encapsulates both the graph-solving and pathfinding methods required to
 * visit every Sensor provided to the Drone in an efficient way. A flight is created each time the
 * public method flyOnSensors() is called, after this, the drone resets itself in preparation
 * for a new flight. The most efficient use of the Drone over multiple SensorLists is to create 
 * a Drone, then use the flyOnSensors() method for each SensorList; the flightPathOutput and 
 * mapOutput is overwritten every flight. The Drone has one sub-class named Movement which it
 * uses to efficiently store the required information every time it moves.
 * 
 * Methods are in the approximate order they will be called in execution.
 * 
 * @see Movement
 */
public class Drone 
{
    
    // Define Drone Movement and Reading Constants:
    private static final double MOVE_DISTANCE = 0.0003; // The distance of one Movement.
    private static final double READ_DISTANCE = 0.0002; // The minimum distance between Drone and Sensor for reading.
    private static final double WAYPOINT_SIZE = 6e-5;   // Defines an area around visited points which the Drone no longer considers.
    private static final int    MOVE_ANGLE    = 10;     // The angle (in degrees) between valid Movements.
    private static final int    MAX_MOVES     = 150;    // The maximum amount of moves the Drone can take.
    
    private Boolean exceededMaxMoves; // A flag to inform the Drone that it has exceeded its move count.
                    
    private int expectedSensors; // The number of sensors the Drone expects to visit.
    
    // The Current Point of the Drone:
    private double latitude;        // The Drone's current latitude.
    private double longitude;       // The Drone's current longitude.
    
    // Define the Drone's "home":
    private String homeString = "home"; // A name to give to the Drone's home.
    private double returnLatitude;      // The Drone's home latitude.
    private double returnLongitude;     // The Drone's home longitude.
    private Point  returnPoint;         // The home Point of the drone.
    
    // Define the Drone's Outputs:
    private String        flightPathOutput; // The flight path the drone undertook across a given List of Sensors.
    private List<Feature> mapOutput;        // The readings and LineString for this List of Sensors.

    // Define Lists Used for Pathfinding:
    private List<Movement>  outputPoints; // The Movements the Drone made flying over a List of Sensors.
    private List<String>    flightPath;   // A List of Sensor locations in the form of WhatThreeWords.
    private List<String>    visited;      // A List of visited Sensors.
    private List<NoFlyZone> noFlyZones;   // A List of areas the Drone cannot move through.
    private SensorList      sensorList;   // A List of Sensors and potential Paths between them.
    

    /**
     * The constructor for the Drone class.
     * 
     * @param startPoint a String array. The first String will be parsed as the
     *                   drones starting latitude, the second String will be parsed as the 
     *                   drones starting longitude. Any further String is ignored.
     */
    public Drone( String[] startPoint ) 
    {
        double lat = Double.parseDouble( startPoint[0] ),
               lon = Double.parseDouble( startPoint[1] ); 
         
         // Set the Drone's Home:
         this.setHome( lon, lat );

         // Move the Drone to Home:
         this.setLatitude ( lat );
         this.setLongitude( lon );
    }
    
    /**
     * The Drone takes the given SensorList and flies around them in 
     * the best found path. It will avoid the given NoFlyZones.
     * The results of the flight will be published to this.mapOutput and 
     * this.flightPathOutput respectively. The Drone will then reset itself
     * in preparation for a new flight.
     * 
     * @param sensorList a SensorList object containing all the Sensors to visit.
     * @param noFlyZones a list of NoFlyZone objects which will define everywhere 
     *                   the Drone cannot go.
     * 
     * @see SensorList
     * @see NoFlyZone
     */
    public void flyOnSensors( SensorList sensorList, List<NoFlyZone> noFlyZones ) 
    {
        // Set the Boolean that tells the Drone to stop moving to false so that it moves:
        this.exceededMaxMoves = false;
        
        // Informs the Drone of the size of the SensorList, which informs List sizes used in execution:
        this.expectedSensors = sensorList.size();
        
        // Initialise the arrays with approximate amount of expected Objects they will contain:
        this.outputPoints = new ArrayList<Movement>( this.expectedSensors * 3 );
        this.flightPath   = new ArrayList<String>  ( this.expectedSensors + 1 );
        this.visited      = new ArrayList<String>  ( this.expectedSensors + 1 );
        this.mapOutput    = new ArrayList<Feature> ( this.expectedSensors + 1 );
        
        // Initialise the flightPath output String.
        this.flightPathOutput = "";
        
        // Initialise the Drone's SensorList and add these to the flightPath:
        this.setSensorList( sensorList );
        sensorList.getSensors().forEach( ( word, sensor ) -> this.flightPath.add( word ) );
        
        // Initialise the Drone's NoFlyZones:
        this.setNoFlyZones( noFlyZones );
        
        // Add the Drone's starting location to the output:
        this.outputPoints.add( new Movement ( this.returnPoint, 0 ) );
        
        // Fly the Drone:
        this.fly();
        
        // Reset the Drone for the next flight:
        this.reset();
    }
    
    /**
     * The Drone takes the given SensorList and flies around them in 
     * the best found path.
     * The results of the flight will be published to this.mapOutput and 
     * this.flightPathOutput respectively. The Drone will then reset itself
     * in preparation for a new flight.
     * 
     * @param sensorList a SensorList object containing all the Sensors to visit.
     * 
     * @see SensorList
     */
    public void flyOnSensors( SensorList sensorList ) 
    {
        // Set the Boolean that tells the Drone to stop moving to false so that it moves:
        this.exceededMaxMoves = false;
        
        // Informs the Drone of the size of the SensorList, which in turn informs List sizes used in execution:
        this.expectedSensors = sensorList.size();
        
        // Initialise the arrays with approximate amount of expected Objects they will contain:
        this.outputPoints = new ArrayList<Movement>( this.expectedSensors * 3 );
        this.flightPath   = new ArrayList<String>  ( this.expectedSensors + 1 );
        this.mapOutput    = new ArrayList<Feature> ( this.expectedSensors + 1 );
        
        // Initialise the flightPath output String.
        this.flightPathOutput = "";

        // Initialise the Drone's SensorList and add these to the flightPath:
        this.setSensorList( sensorList );
        sensorList.getSensors().forEach( ( word, sensor ) -> this.flightPath.add( word ) );
        
        // Add the Drone's starting location to the output:
        this.outputPoints.add( new Movement ( this.returnPoint, 0 ) );
        
        // Fly the Drone:
        this.fly();
        
        // Reset the Drone for the next flight:
        this.reset();
    }
           
    /**
     * Calculates a solution to the SensorList and flies the Drone over the SensorList,
     * reading each sensor as the Drone goes. Results of the flight are published to flightPathOutput. 
     * Results of the Sensor readings and a visualisation of the flight are published to
     * mapOutput.
     */
    private void fly()
    {
        // First, generate an approximate solution using a Nearest-Neighbour Algorithm:
        greedy(); 
        
        // Next, progressively improve the solution using the Two-Opt algorithm:
        twoOpt();

        // this.flightPath now contains the Drone's solution to the SensorList,
        // the below loop will fly the Drone to each Sensor in the order it 
        // calculated:
        for ( String string : this.flightPath ) 
        {
            int numberVisitedBefore = this.mapOutput.size();

            Point target,
                  origin = Point.fromLngLat( this.longitude, this.latitude );
            
            // if the string is the home string, set the target point to the return point:
            if( !string.equalsIgnoreCase( this.homeString ) ) 
            {
                target = this.sensorList.getSensorPoint( string );
            }
            else target = Point.fromLngLat( this.returnLongitude, this.returnLatitude );
            
            // Find a path from origin -> within READ_DISTANCE of target:
            calculatePath( string, target, origin, true );
            
            // If the last path brought the drone over the limit of moves, stop:
            if( exceededMaxMoves ) break;
            
            int numberVisitedAfter = this.mapOutput.size();
            
            // If the Sensor wasn't reached, add an unvisited marker:
            if( numberVisitedBefore != numberVisitedAfter - 1 && !string.equalsIgnoreCase( this.homeString ) ) 
            {
                System.out.println( numberVisitedBefore + " " + numberVisitedAfter );
                this.mapOutput.add( this.sensorList.connectToSensor( string, false ) );
            } 
            
            this.visited.add( string );
        }
        
        // this.outputPoints now contains every Movement the Drone
        // made in it's journey around the SensorList. This for loop
        // translates those Movements into the format asked for in 
        // the coursework documentation:
        for( int i = 1; i < this.outputPoints.size(); i++ )
        {
            int j = i - 1;
            Movement lastMove = this.outputPoints.get(j),
                     thisMove = this.outputPoints.get(i);
                        
            Point from  = lastMove.getPoint(),
                  to    = thisMove.getPoint();
                                                
            int   angle = thisMove.getAngle();
            
            this.flightPathOutput += i + "," + from.longitude() + "," + from.latitude() + ","
                                             + angle + "," 
                                             + to.longitude() + "," + to.latitude() + "," 
                                             + thisMove.getWords() + "\n";
        }
        
        // the following takes the Point from each Movement made and makes a Feature from them:
        List<Point> output = new ArrayList<Point>( this.outputPoints.size() );
        this.outputPoints.forEach( move -> output.add( move.getPoint() ) );
        
        LineString line        = LineString.fromLngLats( output );
        Feature    featureLine = Feature.fromGeometry( line );  
        
        // Add the LineString to the mapOutput:
        this.mapOutput.add( featureLine );  
        
        // The first output should always be expectedSensors + 1.
        // The second is the length of the path.
        System.out.println( mapOutput.size() + " " + ( outputPoints.size() - 1 ) );
        
        // We are Done!
    }
    
    /**
     * Sends the Drone back to its returnPoint and clears its memory in preparation for a new flight.
     */
    private void reset()
    {
        // Move the Drone to Home:
        this.setLatitude ( this.returnLatitude  );
        this.setLongitude( this.returnLongitude );
                
        // Remove the given SensorList and NoFlyZones:
        this.setSensorList( null );
        this.setNoFlyZones( null );
    }
    
    /**
     * Greedy is the informal term referring to the Nearest-Neighbour algorithm. It works
     * by searching through the SensorList for the nearest sensor, adding that to the 
     * solution and repeating until no more Sensors are available. It is good as serving
     * as a baseline for improvement algorithms such as the k-opt algorithms.
     * 
     * @return an int with the sum of the path lengths made to move to each sensor and back to home.
     */
    private int greedy()
    {        
        // initialise an empty list, and an entirely full one:
        List<String> openList    = new ArrayList<String>( this.sensorList.getSensors().keySet() ),
                     closedList  = new ArrayList<String>( this.expectedSensors );
        
        List<String> permutation = new ArrayList<String>( this.expectedSensors );
        
        // Get the nearest Sensor to home to begin:
        String nearest = getNearestSensor( this.returnPoint, openList );  
        openList.remove( nearest );
        permutation.add( nearest );
        
        // While we've still sensor's to visit:
        while( !openList.isEmpty() ) 
        {       
            if( !closedList.isEmpty() ) nearest = closedList.get( closedList.size() - 1 );
                        
            String nextNearest      = getNearestSensor( this.sensorList.getSensorPoint( nearest ), openList );    

            closedList.add ( nextNearest ); 
            openList.remove( nextNearest ); // Don't check for this Sensor again.

            permutation.add( nextNearest );
        }
        
        // set and return the length of the permutation:
        return calculateTourPermutation( permutation, true );
    }
    
    /**
     * The Two-Opt Algorithm works by taking an initial feasible solution to the graph (such as the one made by
     * Drone.greedy()) and iteratively searching for improvements. It will continually attempt to flip sections
     * of the initial solution until no more improvements can be made this way.
     * 
     * @return an int with the sum of the path lengths made to move to each sensor and back to home.
     */
    private int twoOpt()
    {      
        List<String> permutation = this.flightPath,      
                     deepCopy    = new ArrayList<String>();   
        
        int before = calculateTourPermutation( permutation, false );
        
        boolean better = true;
        WHILE: while ( better )
        {
            better = false;
            for( int i = 1; i < permutation.size() - 1; i++ )
            {
                for ( int j = i + 1; j < permutation.size(); j++ )
                {
                    if( deepCopy.size() != permutation.size() ) deepCopy.addAll( permutation );
                    
                    // Reverse the list between i and j:
                    List<String> subList = deepCopy.subList( i, j );
                    Collections.reverse( subList );
                    
                    // Calculate how good this permutation is:
                    int after  = calculateTourPermutation( deepCopy, false );
                    if( after < before )
                    {
                        // set new goal to beat:
                        before = after;
                        
                        // set the permutation:
                        permutation.clear();
                        permutation.addAll( deepCopy );
                        
                        better = true;
                        continue WHILE;
                    }
                    else 
                    {
                        better = false;
                        // reset the list:
                        deepCopy.clear();
                    }
                }
             }
        }
        
        // set and return the length of the permutation:
        return calculateTourPermutation( permutation , true );
      }
    
    /**
     * Returns the nearest Sensor's WhatThreeWords location in the SensorList from the given Point. 
     * "Nearest" is considered the Sensor with the lowest euclidean distance.
     * 
     * @param from       a Point as defined in com.mapbox.geojson.Point, defining the Point to search through
     *                   the SensorList from.
     * @param sensorList a List of Strings corresponding to the Sensors to consider .
     * @return           the nearest Sensor's WhatThreeWords location to the Point.
     */
    private String getNearestSensor( Point from, List<String> sensorList ) 
    {
        // if one item in list, return that item:
        int listSize = sensorList.size();
        if( listSize == 1 ) return sensorList.get(0);
        
        int indexOfNearest = 0;
        
        double minDistance = Double.MAX_VALUE;
        
        // For every Sensor in the list:
        for( int i = 0; i < listSize; i++ )
        {
            String sensorAtIndexI = sensorList.get(i);
            
            Point to = this.sensorList.getSensorPoint( sensorAtIndexI );
            
            double distance = euclideanDistance( to, from );
            
            // if this distance is the lowest yet, set it to the nearest:
            if ( distance < minDistance ) 
            {
                minDistance    = distance;
                indexOfNearest = i;
            }
        }
        return sensorList.get( indexOfNearest );
    }
    
    /**
     * Generates a LineString as defined in com.mapbox.geojson.LineString that connects the two given Points 
     * as defined in com.mapbox.geojson.Point by using an A* variation algorithm. It implements the cost function 
     * f(n) = the euclidean distance to the Point to - the euclidean distance to the Point from, and selects the 
     * Points that minimise this function. 
     * 
     * @param sensor         the location of the sensor to move towards given in WhatThreeWords format.
     * @param targetPoint    the Point to move towards.
     * @param startPoint     the Point to move away from.
     * @param changePosition a Boolean that determines whether the Points found should be added to outputPoints
     * @return               a LineString of all the Points found during the path finding between startPoint and targetPoint including the startPoint.
     */
    private LineString calculatePath( String sensor, Point targetPoint, Point startPoint, Boolean changePosition ) 
    {        
        if( sensor == this.homeString || sensor == null) sensor = "null";
        
        // Defines how many options on the circle of radius MOVE_DISTANCE we can consider:
        int pathOptions = 360 / MOVE_ANGLE;
        
        Point  currentPoint = Point.fromLngLat( 0, 0 );
        
        List<Movement> openList   = new ArrayList<Movement>( pathOptions ),
                       closedList = new ArrayList<Movement>();
                
        Movement currentMove = new Movement ( startPoint, 0 );
        currentMove.setFScore( euclideanDistance( targetPoint, startPoint ) );
        
        Movement firstMove   = currentMove;
        
        openList.add( currentMove ); 

          
        // if the Drone is already within reading range, skip the WHILE loop and move to the 
        // special Movement case: 
        Boolean alreadyInRange = false;
        if( Double.compare(euclideanDistance( startPoint, targetPoint ), READ_DISTANCE ) < 0 ) alreadyInRange = true;
        
        WHILE: while( !openList.isEmpty() && !alreadyInRange ) 
        {
                        
            int openSize  = openList.size(),
                closeSize = closedList.size();

            ArrayList<Double> costFunction = new ArrayList<Double>( openSize );
            
            // Select the best move for the Drone:
            for( Movement move : openList )
            {                
                costFunction.add( move.fScore );
            }
            
            double leastCost      = Collections.min( costFunction );
            int    leastCostIndex = costFunction.indexOf( leastCost );
                   currentMove    = openList.get( leastCostIndex );
                   currentPoint   = currentMove.getPoint();

            // Safety checking, making sure the drone doesn't violate a no-fly-zone:        
            if( closeSize > 1 ) 
            {
                Point lastPoint = closedList.get( closeSize - 1 ).getPoint();
                if( insideNoFlyZones( lastPoint, currentPoint ) )
                {
                    openList.remove( leastCostIndex );
                    continue WHILE;
                }
            }
                  
            // Finished!
            // if within range, reconstruct the path:
            if( Double.compare( euclideanDistance( currentPoint, targetPoint ), READ_DISTANCE ) < 0  )
            {
                // Destroy the list:
                closedList.clear();
                
                // Add the last Movement made 
                closedList.add( currentMove );
                
                // While there is still parent's to get, add them to the path:
                while( currentMove.parent != null )
                {
                    currentMove = currentMove.parent;
                    closedList.add( currentMove );
                }
                
                // Reverse the reconstructed path:
                Collections.reverse( closedList );
                
                break WHILE;
            }
            
            // Not finished.
            // Consider each potential Movement and add valid ones to the openList.
            else 
            {
                // Add the best Movement to the closedList:
                closedList.add( currentMove );
                
                openList.remove( currentMove );
                
                FOR: for( int i = 0; i < pathOptions; i++ ) 
                {
                    int angleDegrees = i * MOVE_ANGLE;
                    
                    double angle = Math.toRadians( angleDegrees ),
                           x     = currentPoint.longitude() + ( MOVE_DISTANCE * Math.cos( angle ) ), 
                           y     = currentPoint.latitude()  + ( MOVE_DISTANCE * Math.sin( angle ) );

                    Point    newPosition = Point.fromLngLat( x, y );                             
                    Movement newMove     = new Movement( newPosition, angleDegrees ); 

                    // In ascending order of computation required to complete.
                    // If the new Point violates any of the rules defined below
                    // do not consider it as a potential Movement.
                    if( !NoFlyZone.insideBasicLimits( newPosition ) )      continue FOR; // Considers if the Point is not in the flight zone.
                    if( insideNoFlyZones   ( newPosition, currentPoint ) ) continue FOR; // Considers if the Point violates a no-fly-zone.           
                    
                    if( pointNearPoints( newPosition, closedList ) ) continue FOR; // Considers if the new position has already been considered:
                    if( pointNearPoints( newPosition, openList   ) ) continue FOR; // As above.      

                    // g-score = number of movements made before this:
                    double g = 0;
                    Movement move = currentMove;
                    if( move.parent != null )
                    {
                        g = move.parent.gScore;
                    }
                    
                    // Here lies the A* heuristic equation, it will try to minimise this function:
                    double distanceToTarget  = euclideanDistance( newPosition, targetPoint );
                    double distanceTravelled = euclideanDistance( newPosition, startPoint  );
                    double fScore = ( g / 2 ) + distanceToTarget - ( distanceTravelled / 3 );
                    
                    // Set the scores and parent move of this Movement:
                    // g score will always equal an integer multiple of MOVE_DISTANCE.
                    newMove.setGScore( g + MOVE_DISTANCE );
                    newMove.setFScore( fScore );
                    newMove.setParent( currentMove );
                    
                    openList.add( newMove );
                }
            } 
        }       
        
        // If the Drone is already within reading range of the sensor
        // move to the first valid position found and return as we must move to read a Sensor.
        // As this is an inefficient Movement, this will rarely show in final path.
        if( alreadyInRange ) 
        {
            closedList.add( currentMove );  
            
            int finalAngle = 0;
            FOR: for( int i = 0; i < pathOptions; i++ )  
            {
                int angleDegrees = i * MOVE_ANGLE;

                currentPoint = currentMove.point;
                
                double angle = Math.toRadians( angleDegrees ),
                       x     = currentPoint.longitude() + ( MOVE_DISTANCE * Math.cos( angle ) ),
                       y     = currentPoint.latitude()  + ( MOVE_DISTANCE * Math.sin( angle ) );
                
                Point  newPosition = Point.fromLngLat( x, y ); 
                
                if( !NoFlyZone.insideBasicLimits( newPosition  ) )  continue FOR; // Considers if the Point is not in the flight zone.
                if( insideNoFlyZones( newPosition, currentPoint ) ) continue FOR; // Considers if the Point violates a no-fly-zone.
                
                finalAngle       = angleDegrees; 
                Movement newMove = new Movement( newPosition, finalAngle );
                closedList.add( newMove );   

                break;
            }
            finalAngle = ( finalAngle + 180 ) % 360;
            Movement goBack = new Movement( currentMove.getPoint() , finalAngle );
            closedList.add( goBack );   

        }
        
        // if the Drone is flying, publish Movements and move to each point:
        if( changePosition ) 
        {
            for( Movement move : closedList )
            {
                // the first move is the end of the last one, so don't publish it:
                if( move != firstMove ) 
                {
                    moveDrone( move, sensor );
                }
            }
        }
        
        
        // Create a LineString of points in the closedList:
        List<Point> outputPoints = new ArrayList<>( closedList.size() );
        closedList.forEach( move -> outputPoints.add( move.getPoint() ) );
        LineString lineString = LineString.fromLngLats( outputPoints );

        return lineString;
    }
    
    /**
     * Moves the Drone to the point defined by the given Movement. If the Drone is
     * within range of the Sensor given in the sensor parameter, it will read and 
     * store the Sensor's details into this Drone's mapOutput.
     * 
     * @param move   a Movement to add to the Drone's output which contains the Point
     *               to move the Drone to.
     * @param sensor the WhatThreeWords location of the sensor the Drone is moving towards.
     */
    private void moveDrone( Movement move, String sensor )
    {      
        int outputSize = this.outputPoints.size();

        // if the Drone has exceeded its MAX_MOVE count, stop the Drone from flying:
        if( outputSize >= MAX_MOVES ) 
        {
            stop();
        }
        
        // if this move is already in the output, don't consider it:
        else if( this.outputPoints.get( outputSize - 1 ) != move )
        {
            this.latitude  = move.latitude();
            this.longitude = move.longitude(); 
            this.outputPoints.add( move );
            
            // if the drone is moving towards a sensor, publish the sensor to output if in range:
            if( sensor != "null" ) {
                Point  sensorPoint = this.sensorList.getSensorPoint( sensor );    
                if( Double.compare( euclideanDistance( move.getPoint(), sensorPoint ), READ_DISTANCE ) < 0 )
                {
                    Feature feature = this.sensorList.connectToSensor( sensor, true );
                    if( !this.mapOutput.contains( feature ) ) {
                        this.mapOutput.add( feature );
                        this.outputPoints.get( this.outputPoints.size() - 1 ).setWords( sensor );
                    }
                }
            }
        } 
    }
    
    /**
     * Does what it says on the tin: stops the Drone publishing any more moves
     * and publishes the unvisited sensors as grey markers into the mapOutput.
     */
    private void stop() 
    {
        // if already stopped, don't stop again:
        if( !this.exceededMaxMoves ) 
        {
            // A for loop that publishes each unvisted Sensor:
            for( String string : this.flightPath )
            {
                if( string != "home" ) 
                {
                    Feature feature = this.sensorList.connectToSensor( string, false );
                    if( !this.visited.contains( string ) ) 
                    {
                        this.mapOutput.add( feature );
                    }
                }
            }
            
            System.err.println("I made too many moves! Flight has stopped to protect battery");
        }
        this.exceededMaxMoves = true;
    }
    
    /**
     * Checks that the given point is not within the "waypoint box" of any of the
     * points within the List of Movements given. The size of the box is defined by
     * this Drone's <code>WAYPOINT_SIZE static double</code>.
     * 
     * @param point a Point to consider against every Movement in moves.
     * @param moves a List of Movements, each of which have a Point to check point against.
     * @return      <code>true</code> if point is within range of a Point within moves,
     *              otherwise <code>false</code>.
     *              
     * @see Point
     */
    private Boolean pointNearPoints( Point point, List<Movement> moves ) 
    {
        double x = point.longitude(),
               y = point.latitude();
        
        for ( int k = 0; k < moves.size(); k++ )
        {
            Point thisPoint = moves.get(k).getPoint();
            
            double lonMin = thisPoint.longitude() - WAYPOINT_SIZE,
                   lonMax = thisPoint.longitude() + WAYPOINT_SIZE,
                   latMin = thisPoint.latitude()  - WAYPOINT_SIZE,
                   latMax = thisPoint.latitude()  + WAYPOINT_SIZE;
            
            if( x > lonMin && x < lonMax && 
                y > latMin && y < latMax  ) return true;
        }
        return false;
    }
             
    /**
     * Returns the approximate number of Movements the Drone must make in order to complete
     * the given permutation. if set is <code>true</code> it will set the Drone's flightPath
     * to the given permutation.
     * 
     * @param permutation a List of Strings that refer to the WhatThreeWords location of each Sensor in order that they will be visited
     * @param set         a Boolean, if set to <code>true</code> the Drone's flightPath will be set to the current permutation.
     * @return            an int with the sum of the path lengths made to move to each sensor and back to home.
     */
    private int calculateTourPermutation ( List<String> permutation, Boolean set )
    {
        SensorList sList = this.sensorList;
        int tourCost = 0;

        Point pI = null,
              pJ = null;
        
        // For every sensor in the permutation, add the length of the path
        // to the tourCost:
        for( int i = 0; i < permutation.size() - 1; i++ )
        {
            int j = i + 1;
            String atI = permutation.get(i),
                   atJ = permutation.get(j);
            
            // get the path between the sensor at i and j:
            var path = sList.getPath( atI, atJ );
            
            if( atI == this.homeString ) pI = this.returnPoint;
            else                         pI = sList.getSensorPoint( atI );
            if( atJ == this.homeString ) pJ = this.returnPoint;
            else                         pJ = sList.getSensorPoint( atJ );

            // if this path hasn't been calculated yet, calculate it:
            if( path == null ) 
            {
                var pathNew = calculatePath( null, pI, pJ, false );
                var pathLength = pathNew.coordinates().size();
                sList.addPath( atI, atJ, pathNew );
                tourCost += pathLength;
            }
            else tourCost += path.coordinates().size();
        }
        
        // if the permutation doesn't contain the flight home, add it in:
        if( !permutation.contains( this.homeString ) ) 
        {
            String last = permutation.get( permutation.size() - 1 );

            LineString path = calculatePath( null, this.returnPoint, this.sensorList.getSensorPoint( last ), false );
            
            sList.addPath( last, this.homeString, path );
            tourCost += path.coordinates().size();
            permutation.add( this.homeString );
        }
         
        // if asked for, set this permutation to the flightPath of the Drone:
        if( set ) this.flightPath = permutation;
        return tourCost;
    }
        
    /**
     * Returns a double that is the length of the line segment between the two given Points.
     * The ordering of the points does not matter. It uses the euclidean distance formula.
     * 
     * @param  pointOne a Point as defined in com.mapbox.geojson.Point.
     * @param  pointTwo a Point as defined in com.mapbox.geojson.Point.
     * @return          the length of the line segment between the two points
     */
    private double euclideanDistance( Point pointOne, Point pointTwo ) 
    {
        double x = Math.pow( pointOne.longitude() - pointTwo.longitude(), 2 ),
               y = Math.pow( pointOne.latitude()  - pointTwo.latitude() , 2 );
        return Math.sqrt( x + y );
    }
           
    /**
     * Evaluates the 2-D line-segment made between the two parameters and determines whether the line violates the given NoFlyZones.
     * 
     * @param  from  a Point, the first  point on the line.
     * @param  to    a Point, the second point on the line.
     * @return       <code>true</code> if the line made between from and to intersects with a NoFlyZone. 
     *               Will always return <code>false</code> if NoFlyZones are given.
     *               
     * @see Point
     * @see NoFlyZone
     */  
    private Boolean insideNoFlyZones( Point from, Point to )
    {
        if( this.noFlyZones == null ) return false;
        
        for( int i = 0; i < this.noFlyZones.size(); i++ )
        {
            NoFlyZone noFlyZoneAtI = this.noFlyZones.get(i);
            if( noFlyZoneAtI.checkIfLineCrossesZone( from, to ) ) 
            {
                return true; 
            }
        }
        
        return false;
    }
     
    // Getters and Setters:    
    public String getFlightPath()
    {
        return this.flightPathOutput;
    }
    
    public FeatureCollection getMapOutput()
    {
        return FeatureCollection.fromFeatures( this.mapOutput );
    }
    
    private void setLatitude( double input )
    {
        this.latitude = input;
    }
    
    private void setLongitude( double input )
    {
        this.longitude = input;
    }
    
    private void setSensorList( SensorList sensorList ) 
    {
        this.sensorList = sensorList;
    }
    
    private void setNoFlyZones( List<NoFlyZone> noFlyZones ) 
    {
        this.noFlyZones = noFlyZones;
    }
        
    private void setHome( double lon, double lat )
    {
        this.returnLatitude  = lat ;
        this.returnLongitude = lon;
        this.returnPoint     = Point.fromLngLat( lon, lat );
    }
    
    /**
     * A Movement encapsulates all the information required to move the Drone one step.
     * Each Movement must have a Point and an angle in the form of an int which refers to 
     * the degrees from East moved.
     * 
     * Additionally, a Movement may have a String referring to a Sensor's WhatThreeWords
     * location, a parent Movement holding the information of the Movement before it, and
     * an f and g score which are used for the A* algorithm.
     * 
     * @see Point
     */
    private class Movement 
    {
        private Movement parent = null;
        private Point    point;  
        private int      angle;
        private String   words;
        private double   fScore = 0;
        private double   gScore = 0;
        
        /**
         * THe sole constructor of the Movement class.
         * 
         * @param in        a Point as defined in com.mapbox.geojson.Point which the Drone has moved to.
         * @param direction an int which is given to the angle from East the drone has moved.
         */
        private Movement( Point in, int direction )
        {
            this.setPoint( in );
            this.setAngle( direction );
        }
        
        private void setGScore( double g ) 
        {
            this.gScore = g; 
        }

        private void setFScore( double f )
        {
            this.fScore = f;
        }

        private void setParent( Movement move )
        {
            this.parent = move;
        }
        
        private Point getPoint() 
        {
            return this.point;
        }
        
        private void setPoint( Point in ) 
        {
            this.point = in;
        }
        
        private int getAngle() 
        {
            return this.angle;
        }
        
        private void setAngle( int angle ) 
        {
            this.angle = angle;
        }
        
        private String getWords() 
        {
            return this.words;
        }

        private void setWords( String words ) 
        {
            this.words = words;
        }
        
        private double latitude()
        {
            return this.point.latitude();
        }
        
        private double longitude()
        {
            return this.point.longitude();
        }

    }
}
