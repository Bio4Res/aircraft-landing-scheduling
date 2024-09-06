package es.uma.lcc.caesium.problem.aircontrol;


import java.io.FileReader;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

import com.github.cliftonlabs.json_simple.JsonArray;
import com.github.cliftonlabs.json_simple.JsonException;
import com.github.cliftonlabs.json_simple.JsonObject;
import com.github.cliftonlabs.json_simple.Jsoner;

import es.uma.lcc.caesium.problem.aircontrol.ea.RunEA4AirControl;
import es.uma.lcc.caesium.problem.aircontrol.grasp.RunGRASP4AirControl;

/**
 * Class for running experiments in batch
 * @author ccottap
 * @version 1.1
 */
public class RunBatch {


	/**
	 * Main method
	 * @param args command-line arguments (name of the bacth file)
	 * @throws JsonException if input file does not have the right format
	 * @throws IOException if files cannot be read or written
	 */
	public static void main(String[] args) throws JsonException, IOException {
		if (args.length < 1) {
			System.out.println("Missing parameters. Required: <batch-conf>");
			System.exit(1);
		}
		// reads batch configuration
		JsonObject conf = (JsonObject) Jsoner.deserialize(new FileReader(args[0]));
		List<String> algorithms = getStrings((JsonArray) conf.get("algorithms"));
		List<String>  instances = getStrings((JsonArray) conf.get("instances"));
		
		String[] params = new String[2];
	
		for (String alg: algorithms) {
			for (String instance: instances) {
				params[0] = alg;
				params[1] = instance;
				System.out.println("--------------------------------------------------------------------------------");
				System.out.println("Running " + params[0] + " " + params[1]);
				System.out.println("--------------------------------------------------------------------------------");
				if (alg.contains("reactive"))
					RunGRASP4AirControl.main(params);
				else
					RunEA4AirControl.main(params);
			}
		}

	}

	/**
	 * Extracts strings from a JSON array
	 * @param jsonArray a json Array
	 * @return the list of strings contained in the array
	 */
	private static List<String> getStrings(JsonArray jsonArray) {
		List<String> l = new LinkedList<String>();
		for (Object o: jsonArray) {
			l.add((String)o);
		}
		return l;
	}

}
