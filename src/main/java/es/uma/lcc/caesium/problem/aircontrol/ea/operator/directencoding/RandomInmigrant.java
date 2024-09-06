package es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding;

import java.util.ArrayList;
import java.util.List;

import es.uma.lcc.caesium.ea.base.Individual;
import es.uma.lcc.caesium.ea.fitness.ObjectiveFunction;
import es.uma.lcc.caesium.ea.operator.variation.mutation.MutationOperator;

/**
 * Random inmigrant: creates a completely new solution.
 * @author ccottap
 * @version 1.0
 *
 */
public class RandomInmigrant extends MutationOperator {
	/**
	 * constructor
	 */
	private RandomFeasibleLandingInformation creator;
	
	/**
	 * Creates the operator. 
	 * @param pars String representation of the mutation probability and the number of neighbors to consider
	 */
	public RandomInmigrant(List<String> pars) {
		super(pars);
		List<String> p = new ArrayList<String>(1);
		p.add("1.0");
		creator = new RandomFeasibleLandingInformation(p); 
	}
	
	@Override
	public void setObjectiveFunction(ObjectiveFunction f) {
		super.setObjectiveFunction(f);
		creator.setObjectiveFunction(f);
	}

	@Override
	protected Individual _apply(List<Individual> parents) {
		Individual ind = creator._apply(null);
		ind.touch();
		return ind;
	}
	

	@Override
	public String toString() {
		return "RandomInmigrant(" + prob + ")";
	}

}
