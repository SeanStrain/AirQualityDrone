package uk.ac.ed.inf.aqmaps;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.net.http.HttpResponse.BodyHandlers;
import java.time.Duration;

/**
 * The FileHandler class holds everything required to connect to and get information
 * from the Webserver given at http://localhost:"port", where "port" is the desired
 * port to connect to. It also allows for writing to new files.
 * 
 * Functions are in order of access modifiers then alphabetical.
 */
public class FileHandler 
{
    private static final int    ARG_LENGTH    = 3;                   // The expected amount of directories between the top directory and the file wished to be read.
    private static final String EXPECTED_PORT = "80";                // The port variable will default to this string unless otherwise set.
    private static final String DIRECTORY     = "http://localhost:"; // The directory of the Webserver.
    
    private static String     port = EXPECTED_PORT; // The port to connect to.
    private static HttpClient client;               // A HttpClient used to connect to the Webserver.
    
    /**
     * A generic utility function.
     * Creates a new file containing the information in the inputString parameter to a
     * file named by the filename parameter.
     * If the file already exists, it will be overwritten.
     *
     * @param  inputString a String that is to be written to the file.
     * @param  filename    a String that holds the name of the output file to be made.
     * @throws IOException if an input/output error occurs during the writing of the data.
     */   
    public static void writeStringToFile( String inputString , String filename ) throws IOException
    {
        var output = new File( filename );
        var writer = new FileWriter( output );
        writer.write( inputString );
        writer.flush();
        writer.close();
    }      
    
    /**
     * Sets the Filehandler's port to the given string.
     * 
     * @param portNumber a String that should contain only numbers.
     */
    static void setPort( String portNumber ) 
    {
        port = DIRECTORY + portNumber;
    }
    
    /**
     * Returns the data written in the files as specified in the coursework document.
     * 
     * @param arguments the directories between the top directory and the file wished to be read.
     * @param type      the type of data stored within the file.
     * @return          a String containing all the information stored in the file.
     */
    static String getDataFromWebserver( String[] arguments, String type )
    {        
        if( arguments.length != ARG_LENGTH ) 
        {
            System.err.println( "Arguments must be a String Array of length " + ARG_LENGTH +
                                 "\nHandler was given a String Array of length " + arguments.length );
            return null;
        } 

        String URL           = port,
               typeLowerCase = type.toLowerCase(),
               pathway       = arguments[0] + "/" + arguments[1] + "/" + arguments[2];
                
        if( typeLowerCase == "words" ) URL += "/words/" + pathway + "/details.json";
        if( typeLowerCase == "dates" ) URL += "/maps/"  + pathway + "/air-quality-data.json";
                       
        if( URL == port ) 
        {
            System.err.println ( "HttpHandler didn't recognise type argument" );
            return null;
        }
        
        return getResponseFromRequest( URL );
    }   
    
    /**
     * Returns the data written in the file "/buildings/no-fly-zones.geojson"
     * 
     * @return a String containing the data at the "/buildings/no-fly-zones.geojson" file.
     */
    static String getNoFlyZones() 
    {        
        String URL = port + "/buildings/no-fly-zones.geojson";
        return getResponseFromRequest( URL );
    }
     
    /**
     * When given a valid URL, this method will return the String contained in the file at the URL.
     * 
     * @param URL a valid URL
     * @return    a String with the information of the file at the URL.
     */
    private static String getResponseFromRequest( String URL ) 
    {
        if( client == null ) client = HttpClient.newBuilder().connectTimeout ( Duration.ofSeconds( 20 ) ).build();
        HttpRequest request = HttpRequest.newBuilder().uri( URI.create( URL ) ).build();
        
        try 
        {
            HttpResponse<String> response = client.send( request, BodyHandlers.ofString() );
            if( response.body().startsWith( "<h1>404" ) ) 
            {
                System.err.println( "File " + request + " Not Found!" );
                return null;
            }
            return response.body();
        } 
        catch( IOException | InterruptedException e ) 
        {
            e.printStackTrace();
        }
        
        System.err.println( "There is an error, request for data failed" );
        return null;
    }   
}