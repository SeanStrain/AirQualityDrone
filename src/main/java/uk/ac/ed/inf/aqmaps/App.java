package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;
import java.util.List;

import org.json.*;

import java.io.IOException;
import java.lang.reflect.Type;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.Point;
import com.mapbox.geojson.Polygon;

import uk.ac.ed.inf.aqmaps.SensorList.Sensor;

/**
 * The App class holds everything required to set up the FileHandler, No-Fly-Zones, Sensors and Drone
 * It is used to command a Drone to fly around the Sensors on a given day.
 * It expects 6 arguments to be passed into its main function as specified in the coursework document,
 * if given these arguments, it will create a reading file and flightpath file for the given date.
 *
 * Methods other than main are in alphabetic order.
 */
public class App  
{                
   
    public static void main( String[] args )
    {
        
        // Place args into variables:
        final String[] date  = {args[2], args[1], args[0]}, // The current date given in the form YYYY-MM-DD
                       start = {args[3], args[4]};          // The starting location of the Drone.
        
        @SuppressWarnings("unused")
        final String seed = args[5]; // a number used for seeded algorithms, currently unused.
        
        final String port = args[6]; // The connection port for the Webserver.
        
        // Set the port of the FileHandler:
        FileHandler.setPort( port );
           
        // Create a new Drone:
        Drone drone = new Drone( start );
        
        // Create the SensorList:
        SensorList sensors = buildSensorList( date );
        
        // Create the NoFlyZones:
        List<NoFlyZone> noFlyZones = buildNoFlyZones();
        
        // Fly the Drone:
        drone.flyOnSensors( sensors, noFlyZones );
        
        for( String string : date ) System.out.print( string + " " );
        
        String mapOut      = drone.getMapOutput().toJson(),
               flightOut   = drone.getFlightPath(),
               filenameMap = "readings-"   + date[2] + "-" + date[1] + "-" + date[0] + ".geojson",
               filenameFly = "flightpath-" + date[2] + "-" + date[1] + "-" + date[0] + ".txt";
        
        // Write to File:
        try 
        {
            FileHandler.writeStringToFile( mapOut, filenameMap );
            FileHandler.writeStringToFile( flightOut, filenameFly );
        } 
        catch ( IOException e ) 
        {
            e.printStackTrace();
        }                     
    }
    
    /**
     * When the FileHandler has a correctly set up WebServer, this function will create
     * a list of No-Fly-Zones as found in the no-fly-zones.txt.
     * 
     * @return          a List of NoFlyZones 
     * @see FileHandler
     * @see NoFlyZone
     */
    private static List<NoFlyZone> buildNoFlyZones()
    {
        
        String JsonNoFlyZones = FileHandler.getNoFlyZones();
        if( JsonNoFlyZones == null   ) return null;
        if( JsonNoFlyZones.isEmpty() ) return null;
        
        FeatureCollection featureCollection = FeatureCollection.fromJson( JsonNoFlyZones );
        List<Feature>     listFeatures      = featureCollection.features();
        List<NoFlyZone>   noFlyZones        = new ArrayList<NoFlyZone>( listFeatures.size() );
        
        for( Feature feature : listFeatures ) 
        {
            Polygon poly = (Polygon) feature.geometry();
            String  name = feature.getStringProperty( "name" );

            List<Point> points       = new ArrayList<Point>( poly.coordinates().get(0) );
            NoFlyZone   newNoFlyZone = new NoFlyZone ( points, name );
            
            noFlyZones.add( newNoFlyZone );
        }  
        
        return noFlyZones;
    }
    
    /**
     * When the FileHandler has a correctly set up WebServer, this function will create
     * a SensorList as found in the Sensor files for the given date.
     * 
     * @param date       a String array in the order YYYY-MM-DD.
     *                   method to create the NoFlyZone objects.
     * @return           a SensorList object.
     * @see FileHandler
     * @see SensorList
     */
    private static SensorList buildSensorList( String[] date ) 
    {
        // Get the Sensor Words and Readings for this day:
        String httpResponse = FileHandler.getDataFromWebserver( date, "dates" ); 
        
        // Generate a List of Sensors:
        Type         listType        = new TypeToken <ArrayList<Sensor>>() {}.getType();
        List<Sensor> sensorArrayList = new Gson().fromJson( httpResponse, listType );
        SensorList   sensorList      = new SensorList( sensorArrayList.size() );
        
        // For each Sensor, add its coordinates:
        for( Sensor sensor : sensorArrayList )
        {            
            String[]   wordsAsArray  = sensor.words().split( "\\." );
            String     httpResponse2 = FileHandler.getDataFromWebserver( wordsAsArray, "words" );
            JSONObject w3wJSON       = new JSONObject( httpResponse2 ).getJSONObject( "coordinates" );
            
            sensor.setLongitude( w3wJSON.getDouble( "lng" ) );
            sensor.setLatitude ( w3wJSON.getDouble( "lat" ) );
            sensorList.addSensor( sensor.words(), sensor );
        }
        
        return sensorList;
    }
}
