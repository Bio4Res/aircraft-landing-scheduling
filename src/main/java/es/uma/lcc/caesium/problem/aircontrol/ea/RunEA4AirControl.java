package es.uma.lcc.caesium.problem.aircontrol.ea;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.Locale;

import com.github.cliftonlabs.json_simple.JsonException;
import com.github.cliftonlabs.json_simple.JsonObject;
import com.github.cliftonlabs.json_simple.Jsoner;

import es.uma.lcc.caesium.ea.base.EvolutionaryAlgorithm;
import es.uma.lcc.caesium.ea.config.EAConfiguration;
import es.uma.lcc.caesium.ea.fitness.DiscreteObjectiveFunction;
import es.uma.lcc.caesium.ea.statistics.EntropyDiversity;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlFlightDecoderObjectiveFunction;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlObjectiveFunction;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlPenaltyObjectiveFunction;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlRunwayDecoderObjectiveFunction;
import es.uma.lcc.caesium.problem.aircontrol.ea.operator.AirControlVariationFactory;

/**
 * Class for testing the evolutionary algorithm for air control
 * @author ccottap
 * @version 1.0
 */
public class RunEA4AirControl {

	/**
	 * Main method
	 * @param args command-line arguments
	 * @throws FileNotFoundException if configuration file cannot be read 
	 * @throws JsonException if the configuration file is not correctly formatted
	 */
	public static void main(String[] args) throws FileNotFoundException, JsonException {
		if (args.length < 2) {
			System.out.println("Parameters: <algorithm-configuration> <problem-data>");
			System.exit(1);
		}
		
		FileReader reader = new FileReader(args[0] + ".json");
		EAConfiguration conf = new EAConfiguration((JsonObject) Jsoner.deserialize(reader));
		conf.setVariationFactory(new AirControlVariationFactory());
		
		int numruns = conf.getNumRuns();
		System.out.println(conf);
		EvolutionaryAlgorithm myEA = new EvolutionaryAlgorithm(conf);
		
		AirControlProblem acp = new AirControlProblem(args[1] + ".acp");
		System.out.println(acp);
		DiscreteObjectiveFunction obj;
		if (args[0].contains("penalty") || args[0].contains("memetic") || args[0].contains("msls"))
			obj = new AirControlPenaltyObjectiveFunction(acp);
		else if (args[0].contains("flight"))
			obj = new AirControlFlightDecoderObjectiveFunction(acp);
		else if (args[0].contains("runway"))
			obj = new AirControlRunwayDecoderObjectiveFunction(acp);
		else
			obj = null;
		myEA.setObjectiveFunction(obj);
		myEA.getStatistics().setDiversityMeasure(new EntropyDiversity());

		for (int i=0; i<numruns; i++) {
			myEA.run();
			System.out.println ("Run " + i + ": " + 
								String.format(Locale.US, "%.2f", myEA.getStatistics().getTime(i)) + "s\t" +
								myEA.getStatistics().getBest(i).getFitness());
			System.out.println(myEA.getStatistics().getBest(i).getGenome());
			System.out.println(acp.formatLandingInformation(((AirControlObjectiveFunction)obj).decode(myEA.getStatistics().getBest(i).getGenome())));
		}
		PrintWriter file = new PrintWriter(args[0] + "-stats-" + args[1] + ".json");
		file.print(myEA.getStatistics().toJSON().toJson());
		file.close();
	}
	
	
	
	

}
