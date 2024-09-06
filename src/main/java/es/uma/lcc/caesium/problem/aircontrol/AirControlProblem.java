package es.uma.lcc.caesium.problem.aircontrol;


import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Scanner;
import java.util.Set;
import java.util.TreeSet;

/**
 * Models the problem of assigning runways and landing slots to airplanes
 * @author ccottap
 * @version 1.2
 */
public class AirControlProblem {
	/**
	 * class-wide RNG
	 */
	private static Random rng = new Random(1);
	
	/**
	 * Sets the seed for the RNG
	 * @param seed the seed for the RNG
	 */
	public static void setSeed (long seed) {
		rng.setSeed(seed);
	}
	
	/**
	 * the flights
	 */
	private Map<String, Flight> flights;
	/**
	 * number of flights
	 */
	private int numFlights;
	/**
	 * number of runways
	 */
	private int numRunways;
	/**
	 * separation constrains
	 */
	private Map<AirplaneType, Map<AirplaneType, Long>> separation;
	/**
	 * number of plane types
	 */
	private static int numTypes = AirplaneType.values().length;
	/**
	 * Earliest arrival time for each plane
	 */
	private Map<String, Long> earliest;
	/**
	 * to refer to flight IDs by number 0..numFlights-1
	 */
	private String[] flightID; 
	/**
	 * to get the number of a flight IDs
	 */
	private Map<String, Integer> flightIndex; 


	/**
	 * Basic constructor. Creates a randomized instance
	 * @param numFlights number of flights
	 * @param numRunways number of runways
	 */
	public AirControlProblem(int numFlights, int numRunways) {
		create(numFlights, numRunways);
		randomize();
		earliest = earliestArrivalTimes(flights);
	}
	
	/**
	 * Constructs an instance from data in a file
	 * @param filename the name of the file
	 * @throws FileNotFoundException if the file cannot be opened
	 */
	public AirControlProblem (String filename) throws FileNotFoundException {
		readFromfile(filename);
		earliest = earliestArrivalTimes(flights);
	}
	
	/**
	 * Returns a map with the earliest arrival time for each flight
	 * @param f the flights
	 * @return a map with the earliest arrival time for each flight
	 */
	private Map<String, Long> earliestArrivalTimes(Map<String, Flight> f) {
		Map<String, Long> eat = new HashMap<String, Long>(f.size());
		for (var e: f.entrySet()) {
			long best = e.getValue().getArrivalTime(0);
			for (int i=1; i<numRunways; i++) {
				long t = e.getValue().getArrivalTime(i);
				if (t < best)
					best = t;
			}
			eat.put(e.getKey(), best);
		}
		return eat;
	}

	/**
	 * Creates the internal data structures
	 * @param numFlights number of flights
	 * @param numRunways number of runways
	 */
	private void create (int numFlights, int numRunways) {
		flights = new HashMap<String, Flight>(numFlights);
		this.numFlights = numFlights;
		this.numRunways = numRunways;
		separation = new HashMap<AirplaneType, Map<AirplaneType, Long>>(numTypes);
		for (AirplaneType t: AirplaneType.values()) {
			separation.put(t, new HashMap<AirplaneType, Long>(numTypes));
		}
	}
	
	/**
	 * Generate indices for referring to flights by an index 0..numFlights-1
	 * or for recovering the flight ID given such an index.
	 */
	private void generateIndices() {
		flightID = new String[numFlights];
		flightIndex = new HashMap<String, Integer>();
		int i = 0;
		for (String id: getFlightIDs()) {
			flightIndex.put(id, i);
			flightID[i++] = id;
		}
	}
	
	/**
	 * Randomizes the data
	 */
	private void randomize() {
		AirplaneType[] types = AirplaneType.values();
		long currentTime = 1;
		for (int i=0; i<numFlights; i++) {
			Flight f = new Flight("IB"+String.format("%03d", i+1), types[rng.nextInt(types.length)]);
			for (int j=0; j<numRunways; j++) {
				f.setArrivalTime(j, currentTime + rng.nextInt(numRunways));
			}
			flights.put(f.getFlightID(), f);
			currentTime += rng.nextInt(numRunways);
		}
		for (AirplaneType t1: AirplaneType.values()) {
			for (AirplaneType t2: AirplaneType.values()) {
				separation.get(t1).put(t2, numTypes + rng.nextLong(numTypes+1));
			}
		}
		generateIndices();
	}
	
	/**
	 * Saves the data to a file
	 * @param filename name of the file
	 * @throws FileNotFoundException if the file cannot be created
	 */
	public void saveToFile (String filename) throws FileNotFoundException {
		PrintWriter file = new PrintWriter(filename);
		file.println(numFlights + "\t" + numRunways);
		for (var e: flights.entrySet()) {
			Flight f = e.getValue();
			file.print(f.getFlightID() + "\t" + f.getType());
			for (int j=0; j<numRunways; j++)
				file.print("\t" + f.getArrivalTime(j));
			file.println();
		}
		for (AirplaneType t1: AirplaneType.values()) {
			for (AirplaneType t2: AirplaneType.values()) {
				file.print(separation.get(t1).get(t2) + "\t");
			}
			file.println();
		}
		file.close();
	}

	/**
	 * Reads an instance from a file
	 * @param filename the name of the file
	 * @throws FileNotFoundException if the file cannot be opened
	 */
	public void readFromfile(String filename) throws FileNotFoundException {
		Scanner reader = new Scanner(new File(filename));
		int numFlights = reader.nextInt();
		int numRunways = reader.nextInt();
		create(numFlights, numRunways);
		for (int i=0; i<numFlights; i++) {
			Flight f = new Flight(reader.next(), AirplaneType.valueOf(reader.next()));
			for (int j=0; j<numRunways; j++)
				f.setArrivalTime(j, reader.nextLong());
			flights.put(f.getFlightID(), f);
		}
		for (AirplaneType t1: AirplaneType.values()) {
			for (AirplaneType t2: AirplaneType.values()) {
				separation.get(t1).put(t2, reader.nextLong());
			}
		}
		reader.close();
		generateIndices();
	}
	

	/**
	 * Returns a set of all flight IDs
	 * @return a set of all flight IDs
	 */
	public Set<String> getFlightIDs() {
		return new TreeSet<String>(flights.keySet());
	}
	
	/**
	 * Returns the flight information of a certain flight
	 * @param id the flight ID
	 * @return the flight information
	 */
	public Flight getFlight(String id) {
		return flights.get(id);
	}

	/**
	 * Returns the number of flights
	 * @return the number of flights
	 */
	public int getNumFlights() {
		return numFlights;
	}


	/**
	 * Return the number of runways
	 * @return the number of runways
	 */
	public int getNumRunways() {
		return numRunways;
	}

	/**
	 * Returns the separation required between two planes depending on their type. 
	 * The leading plane is the one to land first, and the trailing one is the plane that comes afterwards.  
	 * @param leading type of the plane that landed first
	 * @param trailing type of the plane landing immediately afterwards
	 * @return the separation required between two planes depending on their type. 
	 */
	public long getSeparation(AirplaneType leading, AirplaneType trailing) {
		return separation.get(leading).get(trailing);
	}
	
	/**
	 * Returns the numerical index of a flight
	 * @param id a flight id
	 * @return a number in 0..numFlights-1 that identifies the flight indicated. If the if does not correspond 
	 * to any flight, a negative value is returned.
	 */
	public int getFlightIndex(String id) {
		if (!flightIndex.containsKey(id))
			return -1;
		return flightIndex.get(id);
	}
	
	/**
	 * Return the flight ID associated to a numerical index in 0..numFlights-1. An error will happen
	 * if the index supplied is not in this range.
	 * @param index a numerical index in 0..numFlights-1
	 * @return the flight ID associates to this index
	 */
	public String getFlightID (int index) {
		return flightID[index];
	}
	
	/**
	 * Returns the earliest arrival time of a flight 
	 * @param id ID of the flight
	 * @return the earliest arrival time of said flight
	 */
	public long getEarliestArrivalTime (String id) {
		return earliest.get(id);
	}


	/**
	 * Checks if a landing assignment is valid
	 * @param info list of landing info for each flight
	 * @return true iff the landing information is valid
	 */
	public boolean isValid(List<LandingInformation> info) {
		Map<String, Long> wait = waitingTime(info);
		
		for (var e: wait.entrySet())
			if (e.getValue() < 0)
				return false;
		
		return true;
	}
		
	
	/**
	 * Returns a map with the waiting time for all flights (landing time minus earliest arrival time).
	 * If there are planes for which the landing time is not valid (because it is earlier than the earliest
	 * arrival time for that flight and runway, or because the separation constraints are not fulfilled), 
	 * a negative value is used, indicating the total time by which the plane anticipates its arrival. 
	 * If no landing information was provided for a flight, a very large negative value would be used.
	 * If a flight has more than one landing record, only the earliest one is considered.
	 * Non-valid landing flights are not considered when determining separation constraints.
	 * @param info the landing information
	 * @return the waiting time for each flight
	 */
	public Map<String, Long> waitingTime (List<LandingInformation> info) {
		Map<String, Long> wait = new HashMap<String, Long>(numFlights);
		
		info.sort(Comparator.comparing(LandingInformation::time)); // sort flights by landing time
		
		Set <String> ids = new HashSet<String>();	// remaining flights to land
		for (String id: flights.keySet()) {
			ids.add(id);
			wait.put(id, Long.MIN_VALUE);			// default value for flights for which no landing information is available
		}
		
		AirportInformation ai = new AirportInformation(numRunways);
		
		for (LandingInformation li: info) {
			String id = li.flightID();
			if (ids.contains(id)) {	// landing information for a flight still to land
				ids.remove(id);
				long t = li.time();
				int r = li.runway();
				Flight f = flights.get(id);
				AirplaneType p = f.getType();
				long minTime;
				if (ai.getTime(r)<0) { // first plane to use this runway
					minTime = f.getArrivalTime(r);
				}
				else {	// previous landings in this runway. Observe separation constraints
					minTime = Math.max(f.getArrivalTime(r), ai.getTime(r) + separation.get(ai.getType(r)).get(p));
				}
				if (t >= minTime) {	// valid landing time
					ai.land(p, r, t);
					wait.put(id, t - earliest.get(id));
				}
				else { 				// landing time is too early. No landing is recorded
					wait.put(id, t - minTime);
				}
			}
		}

		return wait;
	}
		
	
	/**
	 * Returns a print-friendly version of the list of landing assignments provided
	 * @param info a list of landing assignments
	 * @return a string with a legible display of the landings
	 */
	public String formatLandingInformation (List<LandingInformation> info) {
		String str = "";
		
		for (int i = 0; i < numRunways; i++) 
			str += "Runway #" + (i+1) + "\t";
		str += "\n";
		for (int i = 0; i < numRunways; i++) 
			str += "----------\t";
		str += "\n";
		
		Map<Integer, List<LandingInformation>> li = new HashMap<Integer, List<LandingInformation>> ();
		for (int i = 0; i < numRunways; i++) 
			li.put(i, new LinkedList<LandingInformation>());
		info.sort(Comparator.comparing(LandingInformation::time));
		for (LandingInformation i: info) {
			li.get(i.runway()).add(i);
		}
		for (int i=0; i<info.size();) {
			for (int j=0; j<numRunways; j++) {
				List<LandingInformation> r = li.get(j);
				if (r.size()>0) {
					LandingInformation l = r.get(0);
					str += l.flightID() + " " + l.time() + "(" + (l.time() - earliest.get(l.flightID())) + ")\t";
					r.remove(0);
					i++;
				}
				else
					str += "          \t";
			}
			str += "\n";
		}


		return str;
	}
	
	
	@Override
	public String toString() {
		String str = "#flights: " + numFlights + "\n#runways: " + numRunways + "\n";
		for (var e: flights.entrySet()) {
			Flight f = e.getValue();
			str += f + "\n";
		}
		for (AirplaneType t1: AirplaneType.values())
			str += "\t" + t1;
		str += "\n";
		for (AirplaneType t1: AirplaneType.values()) {
			str += t1;
			for (AirplaneType t2: AirplaneType.values()) {
				str += "\t" + separation.get(t1).get(t2);
			}
			str += "\n";
		}
		
		return str;
	}
	
	
	/**
	 * Restricts a runway so that certain flight types cannot land on it
	 * @param r the runway
	 * @param t the flight type that cannot land on {@code r}
	 */
	private void restrictRunway (int r, AirplaneType t) {
		for (String id: getFlightIDs()) {
			Flight f = getFlight(id);
			if (f.getType() == t) {
				f.setArrivalTime(r, 9999);
			}
		}
	}


	/**
	 * Creates random instances
	 * @param args command-line arguments (not used)
	 * @throws FileNotFoundException is file cannot be read/written
	 */
	public static void main(String[] args) throws FileNotFoundException {		
		int[] sizes = {25, 50, 75, 100, 125};
		int[] runways = {2, 3, 3, 4, 4};
		int num = 3;
		
		for (int i=0; i<sizes.length; i++)
			for (int j=0; j<num; j++) {
				String name;
				AirControlProblem acp = new AirControlProblem (sizes[i], runways[i]+j);
				if (j == (num-1)) {
					acp.restrictRunway(runways[i]+j-1, AirplaneType.HEAVY);
					name = "restricted" + sizes[i] + "-" + (runways[i]+j) + ".acp";
				}
				else
					name = "random" + sizes[i] + "-" + (runways[i]+j) + ".acp";
				System.out.println(acp);
				acp.saveToFile(name);
			}
		
		
	}

}
