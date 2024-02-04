package Libraries;

import jade.core.Agent;

/**
 *
 * @author Ricardo Silva Peres <ricardo.peres@uninova.pt>
 */
public interface ITransport {
    void init(Agent a);
    String[] getSkills();
    boolean executeMove(String origin, String destination, String productID);
}
