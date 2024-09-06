package es.uma.lcc.caesium.problem.aircontrol.grasp;

import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;
import java.util.Locale;

import com.github.cliftonlabs.json_simple.JsonException;
import com.github.cliftonlabs.json_simple.JsonObject;
import com.github.cliftonlabs.json_simple.Jsoner;


import es.uma.lcc.caesium.ea.util.JsonUtil;
import es.uma.lcc.caesium.grasp.base.ReactiveGRASP;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;

/**
 * Class for testing the reactive GRASP for the Aircraft Landing Scheduling Problem
 * @author ccottap
 * @version 1.0
 */
public class RunGRASP4AirControl {

	/**
	 * Main method
	 * @param args command-line arguments
	 * @throws JsonException if the configuration file is not correctly formatted
	 * @throws IOException if a file cannot be read or written
	 */
	@SuppressWarnings("unchecked")
	public static void main(String[] args) throws JsonException, IOException {
		if (args.length < 2) {
			System.out.println("Parameters: <algorithm-configuration> <problem-data>");
			System.exit(1);
		}
		
		FileReader reader = new FileReader(args[0] + ".json");
		JsonObject conf = (JsonObject) Jsoner.deserialize(reader);
		reader.close();
		
		AirControlProblem acp = new AirControlProblem(args[1] + ".acp");
		System.out.println(acp);
		AirControlGRASPObjectiveFunction obj = new AirControlGRASPObjectiveFunction(acp);
		if (conf.containsKey("neighbors"))
			obj.setNumNeighbors(JsonUtil.getInt(conf, "neighbors"));
		else
			obj.setNumNeighbors(0);
		
		ReactiveGRASP myRG = new ReactiveGRASP();
		myRG.setObjectiveFunction(obj);
		
		int numruns = JsonUtil.getInt(conf, "numruns");
		myRG.setSeed(JsonUtil.getLong(conf, "seed"));
		myRG.setNumIters(JsonUtil.getInt(conf, "iterations"));
		myRG.setAmplification(JsonUtil.getDouble(conf, "amplification"));
		myRG.setIterUpdate(JsonUtil.getInt(conf, "update"));
		
		
		for (int i=1; i<acp.getNumRunways(); i++) {
			myRG.addValue(i);
		}
		myRG.setVerbosityLevel(0);
		for (int i=0; i<numruns; i++) {
			myRG.run();
			System.out.println ("Run " + i + ": " + 
								String.format(Locale.US, "%.2f", myRG.getStatistics().getTime(i)) + "s\t" +
								myRG.getStatistics().getBestFitness(i));
			System.out.println(myRG.getStatistics().getBest(i));
			System.out.println(acp.formatLandingInformation((List<LandingInformation>)(myRG.getStatistics().getBest(i))));
		}
		PrintWriter file = new PrintWriter(args[0] + "-stats-" + args[1] + ".json");
		file.print(myRG.getStatistics().toJSON().toJson());
		file.close();
	}
	
	
	
	

}
