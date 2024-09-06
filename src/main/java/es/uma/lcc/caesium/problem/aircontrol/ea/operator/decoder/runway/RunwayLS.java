package es.uma.lcc.caesium.problem.aircontrol.ea.operator.decoder.runway;

import java.util.ArrayList;
import java.util.List;

import es.uma.lcc.caesium.ea.base.Individual;
import es.uma.lcc.caesium.ea.fitness.ObjectiveFunction;
import es.uma.lcc.caesium.ea.operator.variation.mutation.MutationOperator;
import es.uma.lcc.caesium.problem.aircontrol.LandingImprovement;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlRunwayDecoderObjectiveFunction;

/**
 * Local search mutation: an improved solution is sought by changing landing runways for flights.
 * @author ccottap
 * @version 1.0
 *
 */

public class RunwayLS extends MutationOperator {
	/**
	 * number of neighbors to consider during local search
	 */
	private int numNeighbors;
	/**
	 * local search method
	 */
	private LandingImprovement ls;


	/**
	 * Creates the operator. 
	 * @param pars String representation of the mutation probability and the number of neighbors to consider
	 */

	public RunwayLS(List<String> pars) {
		super(pars);
		numNeighbors = Integer.parseInt(pars.get(1));
		ls = new LandingImprovement(numNeighbors); 
	}


	@Override
	public void setObjectiveFunction(ObjectiveFunction f) {
		super.setObjectiveFunction(f);
		ls.setData(((AirControlRunwayDecoderObjectiveFunction)f).getProblemData());
	}

	@Override
	protected Individual _apply(List<Individual> parents) {
		AirControlRunwayDecoderObjectiveFunction p = (AirControlRunwayDecoderObjectiveFunction)obj;
		int numFlights = p.getProblemData().getNumFlights();

		List<LandingInformation> li = new ArrayList<LandingInformation>(numFlights);
		int cost = ls.localSearch(p.decode(parents.get(0).getGenome()), li);	
		p.addExtraCost((double)cost/(double)numFlights - 1); // -1 because the solution is technically evaluated

		Individual ind = new Individual();
		ind.setGenome(p.encode(li));
		ind.touch();
		return ind;

	}


	@Override
	public String toString() {
		return "RunwayInsertLocalSearch(" + prob + ", " + numNeighbors + ")";
	}

}
