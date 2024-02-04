package Product;

import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.core.behaviours.SimpleBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.domain.FIPAException;

import java.util.logging.Level;
import java.util.logging.Logger;

import jade.core.AID;
import jade.lang.acl.ACLMessage;
import jade.proto.ContractNetInitiator;
import jade.proto.AchieveREInitiator;

import java.util.Vector;

import jade.core.Agent;

import java.util.ArrayList;
import java.util.Enumeration;

import Utilities.Constants;
import Utilities.DFInteraction;
import Libraries.IResource;

import static Utilities.Constants.DFSERVICE_TRANSPORT;

/**
 * @author Ricardo Silva Peres <ricardo.peres@uninova.pt>
 */
public class ProductAgent extends Agent {

    ArrayList<String> executionPlan = new ArrayList<>();
    // TO DO: Add remaining attributes required for your implementation

    DFAgentDescription[] dfAgentDescriptions;
    DFAgentDescription[] dfAgentSkills;
    boolean resource_negotiation_done;// para bloquear enquanto funções não terminaram
    String id, next_location, current_location;
    AID agv, transport, my_resource, my_skill;
    int execution_step;

    @Override
    protected void setup() {
        Object[] args = this.getArguments();
        this.id = (String) args[0];
        this.executionPlan = this.getExecutionList((String) args[1]);
        System.out.println("Product launched: " + this.id + " Requires: " + executionPlan);

        // TO DO: Add necessary behaviour/s for the product to control the flow
        // of its own production
        this.current_location = "Source";
        this.resource_negotiation_done = false;
        this.execution_step = 0;
        this.my_resource = null;
        this.my_skill = null;

        this.addBehaviour(new find_resource(this));

    }

    @Override
    protected void takeDown() {
        super.takeDown(); //To change body of generated methods, choose Tools | Templates.
    }


    private ArrayList<String> getExecutionList(String productType) {
        switch (productType) {
            case "A":
                return Utilities.Constants.PROD_A;
            case "B":
                return Utilities.Constants.PROD_B;
            case "C":
                return Utilities.Constants.PROD_C;
        }
        return null;
    }

    /*****contract net*****/
    private class initiator extends ContractNetInitiator {
        public initiator(Agent a, ACLMessage msg) {
            super(a, msg);
            //    this.msg = msg;
        }

        @Override
        protected void handleInform(ACLMessage inform) {
            System.out.println(myAgent.getLocalName() + ": INFORM message received");
            ACLMessage msg = new ACLMessage(ACLMessage.REQUEST); // create a new request message
            next_location = inform.getContent();

            try { //request
                System.out.println("Looking for Transport...");
                DFAgentDescription[] dfAgentDescriptions = DFInteraction.SearchInDFByName("sk_move", myAgent);

                transport = dfAgentDescriptions[0].getName();
                //System.out.println(transport);
                msg.addReceiver(transport);
            }
            catch (FIPAException e){
                throw new RuntimeException(e);
            }

            String location = current_location + "#" + next_location + "&" + id;
            //System.out.println(location);
            msg.setContent(location);

            System.out.println("Sending locations to transport");
            myAgent.addBehaviour(new initiatorFIPA_TA(myAgent, msg));

        }

        @Override
        protected void handleAllResponses(Vector responses, Vector acceptances) {

            System.out.println(myAgent.getLocalName() + ": ALL PROPOSALS received");

            ACLMessage auxMsg = (ACLMessage) responses.get(0);
            ACLMessage reply = auxMsg.createReply();
            // if (auxMsg.getPerformative() == ACLMessage.PROPOSE) {
            System.out.println("Accepting proposal " + reply + " from responder " + auxMsg.getSender());
            my_resource = auxMsg.getSender();

            reply.setPerformative(ACLMessage.ACCEPT_PROPOSAL);
            acceptances.add(reply);

            for (int i = 1; i < responses.size(); i++) {
                ACLMessage auxMsg2 = (ACLMessage) responses.get(i);
                ACLMessage reply2 = auxMsg2.createReply();
                System.out.println("(CFP) REFUSE received from: " + auxMsg.getSender().getLocalName());// deveria aparecer ??
                reply2.setPerformative(ACLMessage.REJECT_PROPOSAL);
                acceptances.add(reply2);
            }

        }


    }

    private class find_resource extends OneShotBehaviour {
        public find_resource(Agent a) {
            super(a);
        }

        public void action() {
            ACLMessage msg = new ACLMessage(ACLMessage.CFP); // create a new CFP message
            try {
                System.out.println("Looking for resource: ");
                dfAgentDescriptions = DFInteraction.SearchInDFByName(executionPlan.get(execution_step), myAgent); // search for agents with the specified name

                for (int i = 0; i < dfAgentDescriptions.length; i++) {
                    msg.addReceiver(dfAgentDescriptions[i].getName()); // add the receivers to the message
                    System.out.println(msg);

                }
                msg.setContent(executionPlan.get(execution_step)); // CFP SKILL
                myAgent.addBehaviour(new initiator(myAgent, msg)); // create a new "initiator" behaviour to send the message


            } catch (FIPAException e) {
                throw new RuntimeException(e);
            }
        }
    }


    /*************Fipa request ********/

    private class initiatorFIPA_TA extends AchieveREInitiator {
        public initiatorFIPA_TA(Agent a, ACLMessage msg) {
            super(a, msg);
        }

        @Override
        protected void handleAgree(ACLMessage agree) {

            System.out.println(myAgent.getLocalName() + ": AGREE message received " + agree.getSender().getLocalName());
        }

        @Override
        protected void handleInform(ACLMessage inform) {
            System.out.println(myAgent.getLocalName() + ": INFORM message received " + inform.getSender().getLocalName());
            //System.out.println(executionPlan.get(execution_step) + " will be executed");

            ACLMessage msg = new ACLMessage(ACLMessage.REQUEST);

            msg.addReceiver(my_resource);
            msg.setContent(executionPlan.get(execution_step));
            myAgent.addBehaviour(new initiatorFIPA_RA(myAgent, msg));

            current_location = next_location;

        }
    }


  private class initiatorFIPA_RA extends AchieveREInitiator {
        public initiatorFIPA_RA(Agent a, ACLMessage msg) {
           super(a, msg);
        }
       @Override
        protected void handleAgree(ACLMessage agree) {
           System.out.println(myAgent.getLocalName() + ": AGREE to execute skill");
       }

        @Override
        protected void handleInform(ACLMessage inform) {
           System.out.println(myAgent.getLocalName() + ": INFORM skill is finished");

            ACLMessage msg = new ACLMessage(ACLMessage.REQUEST);
            try{

                if(executionPlan.size() > execution_step){
                    dfAgentSkills = DFInteraction.SearchInDFByName(executionPlan.get(execution_step), myAgent);

                    for (int i = 0; i < dfAgentDescriptions.length; i++) {
                        msg.addReceiver(dfAgentDescriptions[i].getName()); // add the receivers to the message
                        System.out.println(msg);

                    }
                    msg.setContent(executionPlan.get(execution_step));

                    execution_step++;
                    myAgent.addBehaviour(new find_resource(myAgent));
                }
                else{
                    System.out.println("drop the mic!");
                }
            } catch (FIPAException e){
                throw new RuntimeException(e);

           }

       }
  }

}
