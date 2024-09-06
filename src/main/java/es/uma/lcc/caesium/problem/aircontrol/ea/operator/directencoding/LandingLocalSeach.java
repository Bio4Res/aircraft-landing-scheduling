package es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding;

import java.util.ArrayList;
import java.util.List;

import es.uma.lcc.caesium.ea.base.Individual;
import es.uma.lcc.caesium.ea.fitness.ObjectiveFunction;
import es.uma.lcc.caesium.ea.operator.variation.mutation.MutationOperator;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.LandingImprovement;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlObjectiveFunction;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlPenaltyObjectiveFunction;

/**
 * Local search mutation mutation: an improved solution is sought by changing landing runways for flights.
 * @author ccottap
 * @version 1.0
 *
 */
public class LandingLocalSeach extends MutationOperator {
	/**
	 * number of neighbors to consider during local search
	 */
	private int numNeighbors;
	/**
	 * local search method
	 */
	private LandingImprovement ls;
	/**
	 * the problem instance
	 */
	private AirControlProblem acp;
	
	/**
	 * Creates the operator. 
	 * @param pars String representation of the mutation probability and the number of neighbors to consider
	 */
	public LandingLocalSeach(List<String> pars) {
		super(pars);
		numNeighbors = Integer.parseInt(pars.get(1));
		ls = new LandingImprovement(numNeighbors); 
	}
	
	@Override
	public void setObjectiveFunction(ObjectiveFunction f) {
		super.setObjectiveFunction(f);
		acp = ((AirControlObjectiveFunction)f).getProblemData();
		ls.setData(acp);
	}

	@Override
	protected Individual _apply(List<Individual> parents) {
		AirControlPenaltyObjectiveFunction p = (AirControlPenaltyObjectiveFunction)obj;
		int numFlights = acp.getNumFlights();

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
		return "LocalSearch(" + prob + ", " + numNeighbors + ")";
	}

}
