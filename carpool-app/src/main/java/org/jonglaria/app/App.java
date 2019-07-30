package org.jonglaria.app;

import com.graphhopper.GraphHopper;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.PathWrapper;
import com.graphhopper.util.PointList;
import com.graphhopper.util.Instruction;
import com.graphhopper.util.InstructionList;
import com.graphhopper.util.TranslationMap;
import com.graphhopper.util.Translation;
import com.graphhopper.util.GPXEntry;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.routing.util.EdgeFilter;

import java.util.Locale;
import java.util.List;
import java.util.concurrent.TimeUnit;

import org.jonglaria.distancematrix.DistanceMatrix.Location;
import org.jonglaria.distancematrix.DistanceMatrix.Locations;
import org.jonglaria.distancematrix.DistanceMatrix.Matrix;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileNotFoundException;
import java.io.IOException;

import java.lang.Math;

import java.util.ArrayList;



import com.graphhopper.reader.osm.GraphHopperOSM;

/**
 * Hello world!
 *
 */
public class App 
{
    static final double GSS_LAT = 48.54258;
    static final double GSS_LON = 9.05727;
    static final String MATRIX_FILENAME = "distance_matrix.dat";
    static final String LOCATIONS_FILENAME = "locations.dat";
    static final double AVERAGE_FUEL_CONSUMPTION = 5.5; // l / 100km
    static final double CO2_MULTIPLICATION_FACTOR_GASOLINE = 23.8;
    static final double CO2_MULTIPLICATION_FACTOR_DIESEL = 26.5;
    static ArrayList<ArrayList<ArrayList<Integer>>> bestroutes = new ArrayList<ArrayList<ArrayList<Integer>>>();
    static ArrayList<ArrayList<Double>> bestroutesco2 = new ArrayList<ArrayList<Double>>();
    static ArrayList<ArrayList<Double>> bestroutes_totaldistance = new ArrayList<ArrayList<Double>>();
    static ArrayList<ArrayList<Long>> bestroutes_totaltime = new ArrayList<ArrayList<Long>>();
    private static void calculate_best_routes(double[][] distance_matrix, long[][] time_matrix, ArrayList<Integer> current_list, double currentco2savings, int maxplaces, int maxlocations, double currenttotaldistance, long currenttotaltime){
        if(current_list.size()>maxplaces){
            // We have reached the maximum recursion depth here:
            return;
        }
        int lastlocationindex = current_list.get(current_list.size()-1);
        for(int i=0; i<maxlocations; i++){
            boolean location_used_already = false;
            ArrayList<Integer> new_current_list = new ArrayList<Integer>();
            double new_currentco2savings = currentco2savings;
            double new_currenttotaldistance = currenttotaldistance;
            long new_currenttotaltime = currenttotaltime;
            for(int location_in_use: current_list){
                if(i==location_in_use){
                    location_used_already = true;
                    break;
                }
                new_current_list.add(location_in_use);
            }
            if(location_used_already) continue;
            // calculate CO2 savings in g
            new_currentco2savings += AVERAGE_FUEL_CONSUMPTION * CO2_MULTIPLICATION_FACTOR_GASOLINE * (distance_matrix[i][lastlocationindex]-distance_matrix[lastlocationindex][lastlocationindex])/1000; // division by 1000 because distance_matrix is in m
            new_currenttotaldistance += distance_matrix[i][lastlocationindex];
            new_currenttotaltime += time_matrix[i][lastlocationindex];
            new_current_list.add(i);
            // We don't count the driver, therefore -1:
            int current_number_of_hitchhikers = new_current_list.size()-1;
            boolean found_top_choice = false;
            for(int nthbest=0; nthbest<10; nthbest++){
                if(bestroutesco2.get(nthbest).size() >= current_number_of_hitchhikers){
                    // new_currentco2savings is a negative value by convention
                    if(bestroutesco2.get(nthbest).get(current_number_of_hitchhikers-1)>new_currentco2savings){
                        bestroutesco2.get(nthbest).set(current_number_of_hitchhikers-1, new_currentco2savings);
                        bestroutes_totaldistance.get(nthbest).set(current_number_of_hitchhikers-1, new_currenttotaldistance + distance_matrix[i][i]);
                        bestroutes_totaltime.get(nthbest).set(current_number_of_hitchhikers-1, new_currenttotaltime + time_matrix[i][i]);
                        bestroutes.get(nthbest).set(current_number_of_hitchhikers-1, new_current_list);
                        found_top_choice = true;
                    }
                }
                else{
                    // The way the recursive function is called, this for loop is probably never needed:
                    for(int rr=bestroutesco2.get(nthbest).size(); rr<current_number_of_hitchhikers-2; rr++){
                        bestroutesco2.get(nthbest).add(0.0); // 0 is worse than any real saving value
                        bestroutes_totaldistance.get(nthbest).add(0.0);
                        bestroutes_totaltime.get(nthbest).add((long)0);
                        ArrayList<Integer> placeholder_list = new ArrayList<Integer>();
                        bestroutes.get(nthbest).add(placeholder_list);
                    }
                    // There is no best value yet, so by definition we are better than anything else so far:
                    bestroutesco2.get(nthbest).add(new_currentco2savings);
                    bestroutes_totaldistance.get(nthbest).add(new_currenttotaldistance+distance_matrix[lastlocationindex][lastlocationindex]);
                    bestroutes_totaltime.get(nthbest).add(new_currenttotaltime+time_matrix[lastlocationindex][lastlocationindex]);
                    bestroutes.get(nthbest).add(new_current_list);
                    found_top_choice = true;
                }
                if(found_top_choice) break;
            }
            calculate_best_routes(distance_matrix,time_matrix,new_current_list,new_currentco2savings, maxplaces, maxlocations, new_currenttotaldistance, new_currenttotaltime);
        }
    }
    private static PathWrapper calculate_route(double fromLat, double fromLon, double toLat, double toLon){
        GraphHopper hopper = new GraphHopperOSM().forMobile();
	hopper.setMemoryMapped();
	hopper.setCHEnabled(false);
        hopper.setDataReaderFile("streets_germany.pbf");
        // where to store graphhopper files?
        hopper.setGraphHopperLocation("working_directory");
        hopper.setEncodingManager(EncodingManager.create("car"));
        // the default limits seem to be too low:
        hopper.setMinNetworkSize(2000, 2000);
        hopper.setPreciseIndexResolution(1000);
        // now this can take minutes if it imports or a few seconds for loading
        // of course this is dependent on the area you import
        hopper.importOrLoad();

        // simple configuration of the request object, see the GraphHopperServlet classs for more possibilities.
        GHRequest req = new GHRequest(fromLat, fromLon, toLat, toLon).
            setWeighting("fastest").
            setVehicle("car").
            setLocale(Locale.US);
        GHResponse rsp = hopper.route(req);

        // first check for errors
        if(rsp.hasErrors()) {
            // handle them!
            List<Throwable> errorlist = rsp.getErrors();
            for(Throwable error: errorlist){
                System.out.println( "Error in routing process: \""+error.toString()+"\", sorry :(.");
                System.exit(1);
            }
        }

        // use the best path, see the GHResponse class for more possibilities.
        PathWrapper path = rsp.getBest();
        return path;
    }
    private static void pretty_print_route(PathWrapper path){
        // points, distance in meters and time in millis of the full path
        PointList pointList = path.getPoints();
        double distance = path.getDistance();
        long timeInMs = path.getTime();
        String traveltime = time_pretty_print(timeInMs);
	TranslationMap trMap = new TranslationMap().doImport();
	Locale l = new Locale("de");
	Translation deTR = trMap.getWithFallBack(l);

        InstructionList il = path.getInstructions();
	for(Instruction i: il){
		System.out.println(i.getTurnDescription(deTR));
		long timeTNIInMs = i.getTime();
		String timetillnextinstruction = time_pretty_print(timeTNIInMs);
		double distanceTNI = i.getDistance();
		String distanceS = distance_pretty_print(distanceTNI);
		System.out.println(distanceS + " (" + timetillnextinstruction + ")");
	}

        // or get the result as gpx entries:
        List<GPXEntry> list = il.createGPXList();
        System.out.println( "Distanz: " + distance_pretty_print(distance));
        System.out.println( "Benötigte Zeit: " + traveltime);
    }
    private static String distance_pretty_print(double distance){
            if(distance > 1000){
                    return String.format("%.2f km", distance / 1000);
            }
        else{
            return String.format("%.0f m", distance);
        }
    }
    private static String time_pretty_print(long time_ms){
        if(time_ms < 60 * 1000){
            return String.format("%d s", 
                    TimeUnit.MILLISECONDS.toSeconds(time_ms) 
                    );
        }
        else if(time_ms < 60 * 60 * 1000){
            return String.format("%d min, %d s", 
                    TimeUnit.MILLISECONDS.toMinutes(time_ms),
                    TimeUnit.MILLISECONDS.toSeconds(time_ms) - 
                    TimeUnit.MINUTES.toSeconds(TimeUnit.MILLISECONDS.toMinutes(time_ms))
                    );

        }
        else{
            return String.format("%d h, %d min, %d s", 
                    TimeUnit.MILLISECONDS.toHours(time_ms),
                    TimeUnit.MILLISECONDS.toMinutes(time_ms) -
                    TimeUnit.HOURS.toMinutes(TimeUnit.MILLISECONDS.toHours(time_ms)),
                    TimeUnit.MILLISECONDS.toSeconds(time_ms) -
                    TimeUnit.MINUTES.toSeconds(TimeUnit.MILLISECONDS.toMinutes(time_ms))
                    );

        }

    }
    public static void main( String[] args ) throws Exception
    {
        if(args.length < 1){
            System.err.println("Expect at least one argument (action)");
            System.exit(1);
        }
	String action = args[0];
        String accountname = "";
        boolean locationfileexists = true;
        switch(action){
            case "add":
                {
                    String displayname = "";
                    double fromLat = 0;
                    double fromLon = 0;
                    int offered_places = 0;
                    if(args.length > 5){
                        try{
                            accountname = args[1];
                            displayname = args[2];
                            fromLat = Double.parseDouble(args[3]);
                            fromLon = Double.parseDouble(args[4]);
                            offered_places = Integer.parseInt(args[5]);
                        } catch (NumberFormatException e){
                            System.err.println("Arguments must be double.");
                            System.exit(1);
                        }
                    }
                    else {
                        System.err.println("Expect six arguments (action, accountname, displayname, lattitude, longitude, offered places).");
                        System.exit(1);
                    }
                    Locations.Builder locations = Locations.newBuilder();
                    try {
                        locations.mergeFrom(new FileInputStream(LOCATIONS_FILENAME));
                    } catch (FileNotFoundException e1) {
                        locationfileexists = false;
                    }
                    if(locationfileexists){
                        for(Location l: locations.getLocationsList()){
                            if(l.getAccountname().equals(accountname)){
                                System.err.println("Location for accountname \""+accountname+"\" already stored.");
                                System.exit(1);
                            }
                            else {
                                //System.out.println("Checked \""+ l.getAccountname() +"\"!=\"" + accountname + "\"");
                            }
                        }
                    }
                    /* Add location to file (existing or new) */
                    Location.Builder location = Location.newBuilder();
                    location.setAccountname(accountname);
                    location.setDisplayname(displayname);
                    location.setLattitude(fromLat);
                    location.setLongitude(fromLon);
                    location.setOfferedPlaces(offered_places);
                    locations.addLocations(location);
                    // Save later only when routing does not fail to prevent corruption of data!

                    boolean matrixfileexists = true;
                    Matrix.Builder distance_matrix = Matrix.newBuilder();
                    try {
                        distance_matrix.mergeFrom(new FileInputStream(MATRIX_FILENAME));
                    } catch(FileNotFoundException e2) {
                        matrixfileexists = false;
                    }

                    int locationnumber = locations.getLocationsList().size();
                    System.out.println(String.format("Number of locations: %d",locationnumber));
                    double[][] new_distance_matrix = new double[locationnumber][locationnumber];
                    long[][] new_time_matrix = new long[locationnumber][locationnumber];
                    if(matrixfileexists){
                        System.out.println(String.format("Previous matrix size: %d", (int)Math.sqrt((double)distance_matrix.getTimeMatrixList().size())));
                        int k = 0;
                        for (int i = 0; i < new_distance_matrix.length-1; i++) {
                            for (int j = 0; j < new_distance_matrix[i].length-1; j++) {
                                new_distance_matrix[i][j] = distance_matrix.getDistanceMatrixList().get(k);
                                new_time_matrix[i][j] = distance_matrix.getTimeMatrixList().get(k);
                                k++;
                            }
                        }
                    }
                    //pretty_print_route(path);
                    int l_index = 0;
                    for(Location l: locations.getLocationsList()){
                        if(l_index < locationnumber-1){
                            PathWrapper path_from = calculate_route(fromLat, fromLon, l.getLattitude(), l.getLongitude());
                            PathWrapper path_to = calculate_route(l.getLattitude(), l.getLongitude(), fromLat, fromLon);
                            new_distance_matrix[l_index][locationnumber-1] = path_from.getDistance();
                            new_time_matrix[l_index][locationnumber-1] = path_from.getTime();
                            new_distance_matrix[locationnumber-1][l_index] = path_to.getDistance();
                            new_time_matrix[locationnumber-1][l_index] = path_to.getTime();
                            System.out.println(String.format("%s->%s: %s (%s)",displayname, l.getDisplayname(), distance_pretty_print(new_distance_matrix[l_index][locationnumber-1]), time_pretty_print(new_time_matrix[l_index][locationnumber-1])));
                            System.out.println(String.format("%s->%s: %s (%s)",l.getDisplayname(), displayname, distance_pretty_print(new_distance_matrix[locationnumber-1][l_index]), time_pretty_print(new_time_matrix[locationnumber-1][l_index])));
                        }
                        else{
                            PathWrapper path_to_tuebingen = calculate_route(fromLat, fromLon, GSS_LAT, GSS_LON);
                            new_distance_matrix[locationnumber-1][locationnumber-1] = path_to_tuebingen.getDistance();
                            new_time_matrix[locationnumber-1][locationnumber-1] = path_to_tuebingen.getTime();
                            System.out.println(String.format("%s->Tübingen: %s (%s)",displayname, distance_pretty_print(new_distance_matrix[locationnumber-1][l_index]), time_pretty_print(new_time_matrix[locationnumber-1][l_index])));
                        }
                        l_index++;
                    }
                    Matrix.Builder write_distance_matrix = Matrix.newBuilder();
                    for (int i = 0; i < new_distance_matrix.length; i++) {
                        for (int j = 0; j < new_distance_matrix[i].length; j++) {
                            write_distance_matrix.addDistanceMatrix(new_distance_matrix[i][j]);
                            write_distance_matrix.addTimeMatrix(new_time_matrix[i][j]);
                            System.out.print(String.format("%s ", distance_pretty_print(new_distance_matrix[i][j])));
                        }
                        System.out.println("");
                    }

                    FileOutputStream outfile = new FileOutputStream(LOCATIONS_FILENAME);
                    locations.build().writeTo(outfile);
                    FileOutputStream matrixoutfile = new FileOutputStream(MATRIX_FILENAME);
                    write_distance_matrix.build().writeTo(matrixoutfile);
                }
                break;
            case "bestroutes":
                {
                    if(args.length > 1){
                        accountname = args[1];
                    }
                    else {
                        System.err.println("Expect two arguments (action, accountname).");
                        System.exit(1);
                    }
                    // Check location file:
                    Locations.Builder locations = Locations.newBuilder();
                    try {
                        locations.mergeFrom(new FileInputStream(LOCATIONS_FILENAME));
                    } catch (FileNotFoundException e1) {

                        System.err.println("No location file found under name \""+LOCATIONS_FILENAME+"\"- please add persons first.");
                        System.exit(1);
                    }
                    boolean accountnameexists = false;
                    int offered_places = 0;
                    int ii=0;
                    int locationindex = 0;
                    for(Location l: locations.getLocationsList()){
                        if(l.getAccountname().equals(accountname)){
                            accountnameexists = true;
                            offered_places = l.getOfferedPlaces();
                            locationindex = ii;
                        }
                        ii++;
                    }
                    if(!accountnameexists){
                        System.err.println("Accountname \""+accountname+"\" does not exist in location file - please add person first.");
                        System.exit(1);
                    }
                    Matrix.Builder distance_time_matrix = Matrix.newBuilder();
                    try {
                        distance_time_matrix.mergeFrom(new FileInputStream(MATRIX_FILENAME));
                    } catch(FileNotFoundException e2) {

                        System.err.println("No matrix file found under name \""+MATRIX_FILENAME+"\"- if you added persons it should exist but apparently it does not.");
                        System.exit(1);
                    }
                    int locationnumber = locations.getLocationsList().size();
                    double[][] distance_matrix = new double[locationnumber][locationnumber];
                    long[][] time_matrix = new long[locationnumber][locationnumber];
                    int k = 0;
                    for (int i = 0; i < distance_matrix.length; i++) {
                        for (int j = 0; j < distance_matrix[i].length; j++) {
                            distance_matrix[i][j] = distance_time_matrix.getDistanceMatrixList().get(k);
                            time_matrix[i][j] = distance_time_matrix.getTimeMatrixList().get(k);
                            k++;
                        }
                    }
                    if(offered_places > 0){
                        System.out.println(String.format("Person offers %d places.",offered_places));
                        ArrayList<Integer> current_list = new ArrayList<Integer>();
                        current_list.add(locationindex);
                        for(int i=0; i<10; i++){
                            ArrayList<ArrayList<Integer>> placeholder_list = new ArrayList<ArrayList<Integer>>();
                            bestroutes.add(placeholder_list);
                            ArrayList<Double> placeholder_double_list = new ArrayList<Double>();
                            bestroutesco2.add(placeholder_double_list);
                            ArrayList<Double> placeholder_double_list2 = new ArrayList<Double>();
                            bestroutes_totaldistance.add(placeholder_double_list2);
                            ArrayList<Long> placeholder_long_list = new ArrayList<Long>();
                            bestroutes_totaltime.add(placeholder_long_list);
                        }
                        calculate_best_routes(distance_matrix,time_matrix,current_list,0, offered_places, locationnumber, 0, 0);
                        for(int nplaces=0; nplaces < offered_places; nplaces++){
                            System.out.println(String.format("%d person/s:", nplaces));
                            System.out.println(String.format("-----------", nplaces));
                            for(int nthbest=0; nthbest<10; nthbest++){
                                for(int brl: bestroutes.get(nthbest).get(nplaces)){
                                    System.out.print(String.format("%s->", locations.getLocationsList().get(brl).getDisplayname()));
                                }
                                System.out.println(String.format("Tbg: %.2f kg CO2 %s (+%s) %s (+%s)", bestroutesco2.get(nthbest).get(nplaces)/1000.0, distance_pretty_print(bestroutes_totaldistance.get(nthbest).get(nplaces)), distance_pretty_print(bestroutes_totaldistance.get(nthbest).get(nplaces)-distance_matrix[locationindex][locationindex]), time_pretty_print(bestroutes_totaltime.get(nthbest).get(nplaces)), time_pretty_print(bestroutes_totaltime.get(nthbest).get(nplaces)-time_matrix[locationindex][locationindex])));
                            }
                        }
                    }
                    else{
                        System.out.println(String.format("Person looks for a place."));
                    }

                } 
                break;
            default:
                System.err.println("Unknown action \""+action+"\". Known actions are \"add\"");
                System.exit(1);
                break;
        }
    }
}
