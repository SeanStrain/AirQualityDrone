package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;
import java.util.List;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.Point;
import com.mapbox.geojson.Polygon;

/**
 * The NoFlyZone class handles everywhere the Drone cannot go.
 * It holds the static method <code>insideBasicLimits</code> 
 * that allows the Drone to determine whether or not it is within
 * the limiting area without needing to have a NoFlyZone object.
 * Each instantiated class carries a list of points that defines
 * its boundaries.
 * 
 */
public class NoFlyZone 
{
    // The following variables define the limiting area:
    private static final double MIN_LON = -3.192473; // The minimum longitude.
    private static final double MAX_LON = -3.184319; // The maximum longitude.
    private static final double MIN_LAT = 55.942617; // The minimum latitude.
    private static final double MAX_LAT = 55.946233; // The maximum latitude.
    
    private List<Point>  points;
    private String       name;
    private Feature      feature;
    
    NoFlyZone( List<Point> points, String name ) 
    {
        this.setPoints( points );
        this.setName( name );
    }   
    
    /**
     * Determines whether the given Point is within the
     * limiting area as defined by the static variables of this class. 
     * 
     * @param point the Point to check the co-ordinates of.
     * @return <code>true</code> if point is within the box defined by these limits by comparing the
     *         given point against each corner of that box and determining that is greater than the minima
     *         and less than the maxima. Otherwise, <code>false</code>.
     *         
     *@see Point
     */
    static Boolean insideBasicLimits( Point point ) 
    {        
        double lon = point.longitude(),
               lat = point.latitude();
        
        if ( lon < MAX_LON && lon > MIN_LON && 
             lat < MAX_LAT && lat > MIN_LAT )
        {
            return true;
        }
        
        return false;
    }
    
    /**
     * Evaluates whether a given line segment as defined by the Points from and to violates
     * the boundaries of this NoFlyzone. The order of the parameters is important, the first
     * point must be the origin of the line, and the second must be the end of the line.
     * 
     * @param from a Point denoting the first  point on the line
     * @param to   a Point denoting the second point on the line.
     * @return     <code>true</code> if the line segement violates this NoFlyZone. else
     *             <code>false</code>.
     *             
     *@see Point           
     */
    Boolean checkIfLineCrossesZone( Point from, Point to )
    {        
        for ( int j = 0; j < this.points.size() - 1; j++ )    
        {                   
            int k = j + 1;
            Point  noFlyOne = this.get(j),
                   noFlyTwo = this.get(k);
            
            // if the lines made by the input and this no-fly-zone's border cross, the line from-to
            // violates the no-fly-zone.
            if( checkIfLinesCross( to, from, noFlyOne, noFlyTwo ) )
            {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Evaluates the 2-D lines made between the first two arguments and the last two and determines whether they cross.
     * This is done by checking the orientation of the triplet of points formed by checking each line against each point 
     * of the other line. If the orientation of the first triplet is different from the second and the third triplet is 
     * different from the fourth, these lines must, therefore, intersect at some point.
     * 
     * @param  one   a Point that forms the first  Point of the first line.
     * @param  two   a Point that forms the second Point of the first line.
     * @param  three a Point that forms the first  Point of the second line.
     * @param  four  a Point that forms the second Point of the second line.
     * @return       <code>true</code> if the lines intersect, otherwise <code>false</code>.
     * 
     * @see Point
     */
    private Boolean checkIfLinesCross( Point one, Point two, Point three, Point four )
    {
        int triangleOneOrientation   = orientation( two  , one , three ),
            triangleTwoOrientation   = orientation( two  , one , four  ),
            triangleThreeOrientation = orientation( three, four, two   ),
            triangleFourOrientation  = orientation( three, four, one   );
                
        if( triangleOneOrientation != triangleTwoOrientation && triangleThreeOrientation != triangleFourOrientation )
        {
            return true;
        }
        
        return false;
    }
    
    /**
     * To determine the orientation of a triplet, we consider their gradients. If the gradient of one to two is
     * greater than that of two to three, the points are clockwise. If the gradient of one to two is less than
     * that of two to three, the points are counter-clockwise. If they are equal, the lines are collinear.
     * 
     * @param  one   a Point that forms the first  point in the triangle.
     * @param  two   a Point that forms the second point in the triangle.
     * @param  three a Point that forms the third  point in the triangle.
     * @return       1 if the points are clockwise, -1 if the points are counter-clockwise, 0 if they are collinear.
     * 
     * @see Point
     */
    private int orientation( Point one, Point two, Point three )
    {
        double oneLat = one.latitude(),
               oneLon = one.longitude(),
        
               twoLat = two.latitude(),
               twoLon = two.longitude(),
        
               threeLat = three.latitude(),
               threeLon = three.longitude(),
        
               val = ( ( twoLat - oneLat ) * ( threeLon - twoLon ) ) - ( ( twoLon - oneLon ) * ( threeLat - twoLat ) );
        
        if ( Double.compare( val, 0 ) < 0 ) return 1;
        if ( Double.compare( val, 0 ) > 0 ) return -1;
        else return 0;
    }
       
    
    /**
     * Creates a Feature out of this no-fly-zone. This Feature can be useful for debugging and visualisation.
     * 
     * @return a Feature consisting of one Polygon which constitutes this no-fly-zone's boundaries.
     * 
     * @see Feature
     */
    Feature zoneToFeature()
    {
        if( this.feature != null ) return this.feature;
        
        List<List<Point>> list = new ArrayList<>( 1 );
        
        list.add( this.points );
        
        Polygon poly    = Polygon.fromLngLats ( list );
        Feature feature = Feature.fromGeometry( poly );
        this.feature = feature;
        
        return feature;
    }
    
    
    public String getName() 
    {
        return name;
    }

    private void setPoints( List<Point> points ) 
    {
        this.points = points;
    }
    
    private Point get( int index )
    {
        return this.points.get( index );
    }

    private void setName( String name ) 
    {
        this.name = name;
    }

}
