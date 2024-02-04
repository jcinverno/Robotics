package Transport;

import Resource.ResourceAgent;
import Utilities.Constants;
import Utilities.DFInteraction;
import jade.core.Agent;
import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;
import Libraries.ITransport;
import jade.domain.FIPAException;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.proto.AchieveREResponder;
import jade.domain.FIPAAgentManagement.RefuseException;
import jade.domain.FIPAAgentManagement.FailureException;
import jade.domain.FIPAAgentManagement.NotUnderstoodException;
import Libraries.ITransport;
import java.util.StringTokenizer;
import static Utilities.Constants.ONTOLOGY_MOVE;

/**
 *
 * @author Ricardo Silva Peres <ricardo.peres@uninova.pt>
 */
public class TransportAgent extends Agent {

    String id;
    ITransport myLib;
    String description;
    String[] associatedSkills;
    boolean beingUsed;
    String next_location, current_location;

    @Override
    protected void setup() {
        Object[] args = this.getArguments();
        this.id = (String) args[0];
        this.description = (String) args[1];
        this.beingUsed = false;
        this.current_location = "Source";
        //Load hw lib
        try {
            String className = "Libraries." + args[2];
            Class cls = Class.forName(className);
            Object instance;
            instance = cls.newInstance();
            myLib = (ITransport) instance;
            System.out.println(instance);
        } catch (ClassNotFoundException | InstantiationException | IllegalAccessException ex) {
            Logger.getLogger(TransportAgent.class.getName()).log(Level.SEVERE, null, ex);
        }

        myLib.init(this);
        this.associatedSkills = myLib.getSkills();
        System.out.println("Transport Deployed: " + this.id + " Executes: " + Arrays.toString(associatedSkills));

        // TO DO: Register in DF
        try {
            DFInteraction.RegisterInDF(this,this.associatedSkills, Constants.DFSERVICE_RESOURCE);
            System.out.println("Registered in DF " + this.getLocalName() + "SKILLS " + Arrays.toString(this.associatedSkills));
        } catch (FIPAException ex) {
            Logger.getLogger(ResourceAgent.class.getName()).log(Level.SEVERE, null, ex);
        }

        // TO DO: Add responder behaviour/s
        this.addBehaviour(new TransportAgent.responder_ta(this, MessageTemplate.MatchPerformative(ACLMessage.REQUEST)));
    }

    @Override
    protected void takeDown() {
        super.takeDown();
    }

    /*********Fipa request achieve responder*/

    public class responder_ta extends AchieveREResponder
    {
        public responder_ta (Agent a, MessageTemplate mt){
            super(a,mt);
        }
        protected ACLMessage handleRequest(ACLMessage request) throws NotUnderstoodException, RefuseException
        {
            System.out.println(myAgent.getLocalName() + ": Processing REQUEST for transport message");
            String location = request.getContent();

            int index1 = location.indexOf("#");
            int index2 = location.indexOf("&");

            current_location = location.substring(0, index1);
            next_location = location.substring(index1 + 1, index2);
            id = location.substring(index2 + 1);

            ACLMessage msg = request.createReply();
            msg.setPerformative(ACLMessage.AGREE);

            System.out.println(this.getAgent().getLocalName() + " TA : sent AGREE to " + request.getSender().getLocalName());

            return msg;
        }

        @Override
        protected ACLMessage prepareResultNotification(ACLMessage request, ACLMessage response) throws FailureException{
            System.out.println(myAgent.getLocalName() + ": Preparing RESULT of transport request");

            myLib.executeMove(current_location,next_location, id);

            ACLMessage msg = request.createReply();
            msg.setPerformative(ACLMessage.INFORM);
            //msg.setOntology(ONTOLOGY_MOVE);

            System.out.println(myAgent.getLocalName() + " TA : Performed MOVE operation to: " + request.getSender().getLocalName());
            //beingUsed = false;
            return msg;
        }

    }

}
