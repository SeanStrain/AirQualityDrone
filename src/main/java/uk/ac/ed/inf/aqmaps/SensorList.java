package uk.ac.ed.inf.aqmaps;

import java.util.*;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

/**
 * The SensorList class contains a Map of Sensors and a Map of paths between them.
 * It allows for efficiently finding Sensors, paths and the relevant information
 * within with only the WhatThreeWords location of the Sensors. Furthermore, the
 * SensorList contains all the information about how Sensors are read and how
 * the resultant Features look.
 *
 * It has one subclass, the Sensor.
 * 
 * @see Sensor
 */
public class SensorList 
{
    
    private static final double LOAD_FACTOR = 0.75; // the load factor of the two maps.
    
    // Define RGB String Constants:
    private static final String[] RGB_STRINGS  = {"#00ff00","#40ff00","#80ff00","#c0ff00","#ffc000","#ff8000","#ff4000","#ff0000"}; // The colours that can be given to a Sensor, the lower the reading, the lower the index of the given SYMBOL.
    private static final String   RGB_LOW_BAT  = "#000000";                                                                         // The colour given to Sensors with a battery level below LOW_BATTERY.
    private static final String   RGB_NOT_VIS  = "#aaaaaa";                                                                         // The colour given to unvisited Sensors.
    private static final int      RGB_MAX_VAL  = 255;                                                                               // The maximum value of a colour in RGB format.
    private static final int      RGB_INTERVAL = ( RGB_MAX_VAL + 1 ) / RGB_STRINGS.length;                                          // The interval between which colour in RGB_STRINGS a Sensor is given depending on it's reading.
    
    // Define Symbol Constants:
    private static final String[] SYMBOLS         = {"lighthouse", "danger"};             // The symbols that can be given to a Sensor, the lower the reading, the lower the index of the given SYMBOL.
    private static final String   SYM_BAD_READ    = "cross";                              // The symbol given to Sensors with a battery level below LOW_BATTERY.
    private static final int      SYMBOL_INTERVAL = ( RGB_MAX_VAL + 1 ) / SYMBOLS.length; // The interval between which symbol in SYMBOLS a Sensor is given depending on it's reading.
    private static final double   BAD_READING     = -1;                                   // A placeholder number to designate sensors with low battery levels.
    private static final double   NOT_VISITED     = -2;                                   // A placeholder number to designate sensors that weren't visited.
    private static final double   LOW_BATTERY     = 10.0;                                 // Sensors with a battery level below this number are considered faulty, and are given the Symbol SYM_BAD_READ.
   
    private Map<String, Sensor>     sensors;      // The SensorList's Sensors.
    private Map<String, LineString> paths;        // The SensorList's paths.
    
    /**
     * The sole constructor for the SensorList. 
     * 
     * @param size the expected size of the arrays the SensorList will contain.
     */
    public SensorList( int size )  
    {
        int capacity = (int) Math.ceil( size * LOAD_FACTOR );
        
        this.sensors = new HashMap<String, Sensor>( capacity );
        this.paths   = new HashMap<String, LineString>( capacity * size );        
    }
    
    /**
     * Creates a Feature as defined by com.mapbox.geojson.Feature made from the properties of
     * the Sensor at the WhatThreeWords location given in the words parameter.
     * 
     * @param  key   a String given in WhatThreeWords format.
     * @param  visited if the Sensor wasn't visited, this parameter will be <code>false</code> and the method
     *                 will return a grey marker
     * @return         a Feature consisting of a location given in WhatThreeWords format and an rgb string, 
     *                 a marker colour and symbol as defined by assignSensorSymbol and assignSensorColour.
     */  
    public Feature connectToSensor( String key, Boolean visited )
    {
        Sensor sensor  = this.getSensor( key );     
        
        double lng     = sensor.longitude(),
               lat     = sensor.latitude(),
               battery = sensor.getBattery(),
               reading;
        
        // If unvisited, assign the NOT_VISITED placeholder number to reading:
        if( !visited ) 
        {
            reading = NOT_VISITED;
        }
        
        // If the battery is lower than LOW_BATTERY, assign the placeholder number BAD_READING to reading:
        else if ( Double.compare( battery, LOW_BATTERY ) < 0 ) 
        {
            reading = BAD_READING;
        }
        
        // otherwise, read the reading from the sensor:
        else 
        {
            reading = Double.parseDouble( sensor.getReading() );
        }
       
        Feature feature  = Feature.fromGeometry( Point.fromLngLat( lng, lat ) );

        String markerColour = assignSensorColour( reading ),
               markerSymbol = assignSensorSymbol( reading );
        
        feature.addStringProperty( "location"     , key          );
        feature.addStringProperty( "rgb-string"   , markerColour );
        feature.addStringProperty( "marker-color" , markerColour );
        
        if( markerSymbol != null ) 
        {
            feature.addStringProperty( "marker-symbol", markerSymbol );
        }
        
        return feature;
    }
    
    /**
     * Returns a String that is of the hexadecimal encoding of a colour as defined in the static constant RGB_STRINGS.
     * The reading argument must be given between 0 and RGB_MAX_VAL.
     *
     * @param  reading the reading from the sensor.
     * @return         the String at the associated index in RGB_STRING assigned to this reading.
     */     
    private String assignSensorColour( double reading ) 
    {     
        if( Double.compare( reading, BAD_READING ) == 0 ) 
        {
            return RGB_LOW_BAT;
        }
        
        if( reading >= 0 && reading <= RGB_MAX_VAL ) 
        {
            int x = (int) Math.floor( reading / RGB_INTERVAL );
            return RGB_STRINGS[x];
        }
      
        if( Double.compare( reading, NOT_VISITED ) == 0 )
        {
            return RGB_NOT_VIS;
        }
        
        return null;
    }
    
    /**
     * Returns a String that is of the a symbol defined in the static constant SYMBOLS.
     * The reading argument must be given between 0 and RGB_MAX_VAL.
     *
     * @param  reading the reading from the sensor.
     * @return         the String at the associated index in SYMBOLS assigned to this reading.
     */     
    private String assignSensorSymbol( double reading ) 
    {
        if( Double.compare ( reading, NOT_VISITED ) == 0 )
        {
            return null;
        }
        
        if( Double.compare ( reading, BAD_READING ) == 0 ) 
        {
            return SYM_BAD_READ;
        }
        
        int x = (int) Math.floor( reading / SYMBOL_INTERVAL );
        return SYMBOLS[x];
    }
    
    //
    // Utility methods, getters and setters:
    //
    public int size() 
    {
        return this.sensors.size();
    }
    
    Point getSensorPoint( String key )
    {
        return this.getSensor( key ).getPoint();
    }
        
    Sensor getSensor( String key )
    {
        return this.sensors.get( key );
    }
            
    void addSensor( String words, String reading, double battery, double longitude, double latitude ) 
    {
        Sensor sensor = new Sensor ( words, reading, battery, longitude, latitude );
        this.sensors.putIfAbsent( words, sensor );
    }
    
    void addSensor( String words, Sensor sensor )
    {
        this.sensors.putIfAbsent( words, sensor );
    }
    
    void removeSensor( String key ) 
    {
        this.sensors.remove( key );
    }
           
    Map<String, Sensor> getSensors() 
    {
        return this.sensors;
    }
    
    Collection<Sensor> getCollectionOfSensors()
    {
        return this.sensors.values();
    }
    
    void addPath( String source, String destination, LineString path )
    {
        String key = source + "-" + destination;
        this.paths.putIfAbsent( key, path );
    }
    
    void removePath( String source, String destination ) 
    {
        String key = source + "-" + destination;
        this.paths.remove( key );
    }
    
    LineString getPath( String source, String destination )
    {
        String key = source + "-" + destination;
        return this.paths.get(key);
    }
    
    Map<String, LineString> getPaths()
    {
        return this.paths;
    }
    
    int getPathLength( String source, String destination ) 
    {
        String key = source + "-" + destination;
        return this.paths.get( key ).coordinates().size();
    }
    
    /**
     * The Sensor class holds all the information a single Sensor needs as specified
     * in the coursework documentation.
     */
    class Sensor 
    {
        private String location;
        private String reading;
        private double battery;
        private double longitude;
        private double latitude;
        private Point  point;

        /**
         * The sole constructor for the Sensor class.
         * 
         * @param location  the WhatThreeWords location of the Sensor.
         * @param reading   the current pollution reading of the Sensor.
         * @param battery   the current battery level of the Sensor.
         * @param longitude the longitudinal position of the Sensor.
         * @param latitude  the latitudinal  position of the Sensor.
         */
        Sensor ( String location, String reading, double battery, double longitude, double latitude ) 
        {
            this.location  = location;
            this.reading   = reading;
            this.battery   = battery;
            this.longitude = longitude;
            this.latitude  = latitude;
            this.point     = Point.fromLngLat( longitude, latitude );
        }
        
        public String words() 
        {
            return this.location;
        }
        
        /**
         * Prints all the information about this sensor, useful for debugging purposes.
         */
        public void printMe() 
        {
            System.out.println ( this.location + " " + this.reading + " " + this.battery + " " +
                                 this.longitude + " " + this.latitude );
        }
        
        void setLongitude ( double longitude )
        {
            this.longitude = longitude;
        }
        
        void setLatitude ( double latitude )
        {
            this.latitude = latitude;
        }
        
        private Point getPoint()
        {
            if( this.point == null ) 
            {
                this.point = Point.fromLngLat( this.longitude, this.latitude );
            }
            return this.point;
        }
        
        private String getReading()
        {
            return this.reading;
        }
        
        private double getBattery() 
        {
            return this.battery;
        }
        
        private double longitude()
        {
            return this.longitude;
        }
        
        private double latitude()
        {
            return this.latitude;
        }
    }
}
