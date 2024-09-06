package es.uma.lcc.caesium.problem.aircontrol.ea.operator;

import java.util.List;

import es.uma.lcc.caesium.ea.operator.variation.VariationFactory;
import es.uma.lcc.caesium.ea.operator.variation.VariationOperator;
import es.uma.lcc.caesium.problem.aircontrol.ea.operator.decoder.runway.RunwayLS;
import es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding.GRASPInitialization;
import es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding.LandingMutation;
import es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding.LandingRecombination;
import es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding.RandomFeasibleLandingInformation;
import es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding.RandomInmigrant;
import es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding.LandingLocalSeach;

/**
 * User-defined factory for the Air Control Problem
 * @author ccottap
 * @version 1.0
 */
public class AirControlVariationFactory extends VariationFactory {

	@Override
	public VariationOperator create (String name, List<String> pars) {
		VariationOperator op = null;
				
		switch (name.toUpperCase()) {
		case "RANDOMLANDING":
			op = new RandomFeasibleLandingInformation(pars);
			break;
		case "RANDOMINMIGRANT":
			op = new RandomInmigrant(pars);
			break;

		case "GRASP":
			op = new GRASPInitialization(pars);
			break;
		case "LANDINGRECOMBINATION":
			op = new LandingRecombination(pars);
			break;
		case "LANDINGMUTATION":
			op = new LandingMutation(pars);
			break;
		case "LANDINGIMPROVEMENT":
			op = new LandingLocalSeach(pars);
			break;
		case "LS-RUNWAY":
			op = new RunwayLS(pars);
			break;

		default:
			op = super.create(name, pars);
		}
				
		return op;
	}

}
