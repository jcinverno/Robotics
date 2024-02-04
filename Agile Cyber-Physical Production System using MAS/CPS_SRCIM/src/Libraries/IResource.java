package Libraries;

import jade.core.Agent;

/**
 *
 * @author Ricardo Silva Peres <ricardo.peres@uninova.pt>
 */
public interface IResource {
    void init(Agent myAgent);
    String[] getSkills();
    boolean executeSkill(String skillID);
}
