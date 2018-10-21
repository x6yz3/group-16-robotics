
/**
 * COMP329 Assignment2
 * 
 * Author: Yuan_Zhu
 * ID: 201220136
 */

import jason.asSyntax.*;
import jason.environment.Environment;
import jason.environment.grid.GridWorldModel;
import jason.environment.grid.GridWorldView;
import jason.environment.grid.Location;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.Stack;
import java.util.logging.Logger;

public class RobotEnv extends Environment {

	/** MAP INFORMATIONS */
	public static ArrayList<Location> psbVics = new ArrayList<>();
	public static ArrayList<Location> obstacles = new ArrayList<>();

	public static Location redLoc = new Location(1, 1);
	public static Location blueLoc = new Location(1, 6);
	public static Location greenLoc = new Location(6, 6);

	/** OBJECTS */
	public static final int G_WID = 8;
	public static final int G_LEN = 8;

	public static final int P_PSB = 256;
	public static final int P_RED = 128;
	public static final int P_BLUE = 64;
	public static final int P_GREEN = 32;
	public static final int P_FAKE = 16;

	public static final int TOKEN = 8;

	public static final int OBS = 4;

	/** COUNTER */
	public int step = 0;

	/** FIND CURRENT LOCATION */
	// robot state information
	public int scoutHead;
	public Location scoutLoc;

	// id found specific location
	public Boolean found = false;

	// all possible locations and headings of the scout of this turn and next turn
	public ArrayList<Location> currPsbLoc = new ArrayList<Location>();
	public ArrayList<Location> nextPsbLoc = new ArrayList<Location>();
	Integer[][][] currPsbHead = new Integer[G_WID][G_LEN][4];
	Integer[][][] nextPsbHead = new Integer[G_WID][G_LEN][4];

	/** FIND VICTIM */
	// next route
	public Integer[][][] route = new Integer[G_WID][G_LEN][2];

	// how many victims found yet
	public int findVicNum = 0;
	public Location nearestVicLoc = new Location(99, 99);
	public Stack<Location> nearestVicRoute = new Stack<Location>();

	// arrive a possible victim
	Boolean arrive = false;
	// find all real victims
	Boolean findAllVic = false;

	// how many possible location left
	public int remainLocNum = 100;

	/** SIMULATION */
	ArrayList<Boolean[]> sm = new ArrayList<Boolean[]>();

	/** JASON PLANS AND PERCEPTIONS */
	// goals
	public static final Term trim = Literal.parseLiteral("trimLoc(slot)");
	public static final Term pick = Literal.parseLiteral("pickNextVic(slot)");
	public static final Term find = Literal.parseLiteral("findNearestRoute(slot)");
	public static final Term goTo = Literal.parseLiteral("goToNearestVic(slot)");
	public static final Term ifVc = Literal.parseLiteral("checkIfVic(slot)");
	public static final Term left = Literal.parseLiteral("checkLeftVic(slot)");
	public static final Term delS = Literal.parseLiteral("deleteScoutAndPsbVic(slot)");

	// perceptions
	public static final Literal onlyLoc = Literal.parseLiteral("onlyLocation");
	public static final Literal noLeftVic = Literal.parseLiteral("noLeftVic");
	public static final Literal atVic = Literal.parseLiteral("atVic");

	// log
	static Logger logger = Logger.getLogger(RobotEnv.class.getName());
	private RobotModel model;
	private RobotView view;

	/** initial environment */
	@Override
	public void init(String[] args) {
		model = new RobotModel();
		view = new RobotView(model);
		model.setView(view);

		Simulation s = new Simulation();
		sm = s.returnSimu();
		logger.info("Total simulation step: " + sm.size());

		updatePercepts();
	}

	/** connect with agent to conduct action */
	@Override
	public boolean executeAction(String ag, Structure action) {

		try {
			Thread.sleep(500);
		} catch (InterruptedException e1) {
			e1.printStackTrace();
		}

		logger.info(ag + " doing: " + action);
		try {

			if (action.equals(trim)) {
				model.trimPsbLoc();
			} else if (action.equals(pick)) {
				model.pickNextVic();
			} else if (action.equals(find)) {
				model.findNearestRoute(nearestVicLoc);
			} else if (action.equals(goTo)) {
				model.goToNearestVic();
			} else if (action.equals(ifVc)) {
				model.checkIfVic();
			} else if (action.equals(left)) {
				model.checkLeftVic();
			} else if (action.equals(delS)) {
				model.deleteScout();
			}

		} catch (Exception e) {
			e.printStackTrace();
		}

		// update agent perceptions each time
		updatePercepts();

		informAgsEnvironmentChanged();
		return true;
	}

	/** creates the agents perception based on the RobotModel each time */
	void updatePercepts() {

		clearPercepts();

		// Location r1Loc = model.getAgPos(0);
		// Literal pos1 = Literal.parseLiteral("pos(r1," + r1Loc.x + "," + r1Loc.y +
		// ")");
		// addPercept(pos1);

		// if there is only one possible location left, assign it to scout agent
		if (remainLocNum == 1) {
			addPercept(onlyLoc);
			scoutLoc = nextPsbLoc.get(0);
			model.setAgPos(1, scoutLoc);
			for (int i = 0; i < 4; i++) {
				if (nextPsbHead[scoutLoc.x][scoutLoc.y][i] == 1) {
					scoutHead = i;
				}
			}
			logger.info("Find location. Location: (" + scoutLoc.x + "," + scoutLoc.y + ") " + " Heading: " + scoutHead);
			remainLocNum = 0;
			found = true;
		}

		if (arrive) {
			addPercept(atVic);
		}

		if (findAllVic) {
			addPercept(noLeftVic);
		}

	}

	class RobotModel extends GridWorldModel {

		private RobotModel() {
			super(G_WID, G_LEN, 2);

			// initial location of agents
			try {
				// doctor agent
				setAgPos(0, 0, 0);
				// scout agent location will be set later
			} catch (Exception e) {
			}

			// initial location of obstacle and wall
			addWall(0, 0, G_WID - 1, 0);
			addWall(0, 1, 0, G_LEN - 1);
			addWall(G_WID - 1, 1, G_WID - 1, G_LEN - 1);
			addWall(1, G_LEN - 1, G_WID - 2, G_LEN - 1);

			// initial location of obstacles
			obstacles.add(new Location(2, 5));
			obstacles.add(new Location(4, 2));
			obstacles.add(new Location(5, 2));
			obstacles.add(new Location(5, 3));
			obstacles.add(new Location(5, 4));
			for (Location l : obstacles) {
				add(OBS, l);
			}

			// initial location of possible victim
			psbVics.add(new Location(1, 1));
			psbVics.add(new Location(1, 6));
			psbVics.add(new Location(6, 1));
			psbVics.add(new Location(6, 6));
			psbVics.add(new Location(4, 5));
			for (Location l : psbVics) {
				add(P_PSB, l);
			}

			// add all possible location to a set
			initPsbLoc();
		}

		/** Initial possible location */
		void initPsbLoc() {

			// initial possible location
			for (int i = 0; i < G_WID; i++) {
				for (int j = 0; j < G_LEN; j++) {
					if (isFreeOfObstacle(i, j) && !hasObject(P_PSB, i, j) && !hasObject(P_RED, i, j)
							&& !hasObject(P_GREEN, i, j) && !hasObject(P_BLUE, i, j) && !hasObject(P_FAKE, i, j)) {
						Location loc = new Location(i, j);
						nextPsbLoc.add(loc);
						add(TOKEN, i, j);
					}
				}
			}

			// initial possible head
			for (int i = 0; i < G_WID - 1; i++) {
				for (int j = 0; j < G_LEN - 1; j++) {
					nextPsbHead[i][j][0] = 1;
					nextPsbHead[i][j][1] = 1;
					nextPsbHead[i][j][2] = 1;
					nextPsbHead[i][j][3] = 1;
				}
			}

			remainLocNum = nextPsbLoc.size();
		}

		/** Connect to robot distance sensor */
		Boolean[] checkAround() throws Exception {
			Boolean[] envArd = new Boolean[4];
			// real action should be input sensor result
			// envArd[0] = front;
			// envArd[1] = left;
			// envArd[2] = right;
			// envArd[3] = back;
			envArd = sm.get(step);
			return envArd;
		}

		/** Reduce scout possible location number */
		void trimPsbLoc() throws Exception {

			// update real information
			Boolean[] envArd = checkAround();

			// update believe information
			currPsbLoc = nextPsbLoc;
			nextPsbLoc = new ArrayList<Location>();
			currPsbHead = nextPsbHead;
			nextPsbHead = new Integer[G_WID][G_LEN][4];
			for (int i = 1; i < G_WID - 1; i++) {
				for (int j = 1; j < G_LEN - 1; j++) {
					nextPsbHead[i][j][0] = 0;
					nextPsbHead[i][j][1] = 0;
					nextPsbHead[i][j][2] = 0;
					nextPsbHead[i][j][3] = 0;
				}
			}

			// for each location in each condition, if believe and real information is
			// equal, this location is possible
			// first delete all possible location, then add then back if possible

			// delete
			for (int i = 0; i < currPsbLoc.size(); i++) {
				Location loc = currPsbLoc.get(i);
				model.remove(TOKEN, loc);
			}
			logger.info("remove " + currPsbLoc.size());

			// add
			for (int i = 0; i < currPsbLoc.size(); i++) {
				Location loc = currPsbLoc.get(i);

				// the robot may have four heading so discuss them all
				// point up
				if (currPsbHead[loc.x][loc.y][0] == 1) {
					if ((model.hasObject(OBS, loc.x, loc.y - 1) == envArd[0])
							&& (model.hasObject(OBS, loc.x - 1, loc.y) == envArd[1])
							&& (model.hasObject(OBS, loc.x + 1, loc.y) == envArd[2])
							&& (model.hasObject(OBS, loc.x, loc.y + 1) == envArd[3])) {
						// if can go straight ahead, do
						if (envArd[0] == false) {
							nextPsbHead[loc.x][loc.y - 1][0] = 1;
							// then if can turn left, do
						} else if (envArd[1] == false) {
							nextPsbHead[loc.x - 1][loc.y][1] = 1;
							// then if can turn right, do
						} else if (envArd[2] == false) {
							nextPsbHead[loc.x + 1][loc.y][2] = 1;
							// then go back
						} else {
							nextPsbHead[loc.x][loc.y + 1][3] = 1;
						}
					}
				}

				// point left
				if (currPsbHead[loc.x][loc.y][1] == 1) {
					if ((model.hasObject(OBS, loc.x - 1, loc.y) == envArd[0])
							&& (model.hasObject(OBS, loc.x, loc.y + 1) == envArd[1])
							&& (model.hasObject(OBS, loc.x, loc.y - 1) == envArd[2])
							&& (model.hasObject(OBS, loc.x + 1, loc.y) == envArd[3])) {
						if (envArd[0] == false) {
							nextPsbHead[loc.x - 1][loc.y][1] = 1;
						} else if (envArd[1] == false) {
							nextPsbHead[loc.x][loc.y + 1][3] = 1;
						} else if (envArd[2] == false) {
							nextPsbHead[loc.x][loc.y - 1][0] = 1;
						} else {
							nextPsbHead[loc.x + 1][loc.y][2] = 1;
						}
					}
				}

				// point right
				if (currPsbHead[loc.x][loc.y][2] == 1) {
					if ((model.hasObject(OBS, loc.x + 1, loc.y) == envArd[0])
							&& (model.hasObject(OBS, loc.x, loc.y - 1) == envArd[1])
							&& (model.hasObject(OBS, loc.x, loc.y + 1) == envArd[2])
							&& (model.hasObject(OBS, loc.x - 1, loc.y) == envArd[3])) {
						if (envArd[0] == false) {
							nextPsbHead[loc.x + 1][loc.y][2] = 1;
						} else if (envArd[1] == false) {
							nextPsbHead[loc.x][loc.y - 1][0] = 1;
						} else if (envArd[2] == false) {
							nextPsbHead[loc.x][loc.y + 1][3] = 1;
						} else {
							nextPsbHead[loc.x - 1][loc.y][1] = 1;
						}
					}
				}

				// point down
				if (currPsbHead[loc.x][loc.y][3] == 1) {
					if ((model.hasObject(OBS, loc.x, loc.y + 1) == envArd[0])
							&& (model.hasObject(OBS, loc.x + 1, loc.y) == envArd[1])
							&& (model.hasObject(OBS, loc.x - 1, loc.y) == envArd[2])
							&& (model.hasObject(OBS, loc.x, loc.y - 1) == envArd[3])) {
						if (envArd[0] == false) {
							nextPsbHead[loc.x][loc.y + 1][3] = 1;
						} else if (envArd[1] == false) {
							nextPsbHead[loc.x + 1][loc.y][2] = 1;
						} else if (envArd[2] == false) {
							nextPsbHead[loc.x - 1][loc.y][1] = 1;
						} else {
							nextPsbHead[loc.x][loc.y - 1][0] = 1;
						}
					}
				}
			}

			// for each location next turn, if at least one of four headings of this
			// location is possible, then this location is possible, so add the location to
			// nextPsbLoc
			for (int i = 1; i < G_WID - 1; i++) {
				for (int j = 1; j < G_LEN - 1; j++) {
					if (nextPsbHead[i][j][0] == 1 || nextPsbHead[i][j][1] == 1 || nextPsbHead[i][j][2] == 1
							|| nextPsbHead[i][j][3] == 1) {
						nextPsbLoc.add(new Location(i, j));
						model.add(TOKEN, new Location(i, j));
					}
				}
			}

			remainLocNum = nextPsbLoc.size();
			step++;

		}

		/** Check how many victims have been found */
		void checkLeftVic() throws Exception {
			// check if all victim has been found
			logger.info(" * found " + findVicNum + " victims");
			if (findVicNum >= 3) {
				findAllVic = true;
			}
		}

		/** Follow optimal path to go one grid */
		void goToNearestVic() throws Exception {
			// use A* to find minimum route to next possible victim and go there
			if (scoutLoc.x == nearestVicLoc.x && scoutLoc.y == nearestVicLoc.y) {
				arrive = true;
			} else {
				Location next = nearestVicRoute.pop();
				// scout just go to neighbour location
				scoutLoc = next;
				model.setAgPos(1, scoutLoc);
				step++;
			}

		}

		/** Check if this victim is real */
		void checkIfVic() throws Exception {

			arrive = false;
			model.remove(P_PSB, scoutLoc);

			if (scoutLoc.x == redLoc.x && scoutLoc.y == redLoc.y) {
				model.add(P_RED, redLoc);
				findVicNum++;
				logger.info(" * find RED victim at: (" + redLoc.x + "," + redLoc.y + ")");
			} else if (scoutLoc.x == blueLoc.x && scoutLoc.y == blueLoc.y) {
				model.add(P_BLUE, blueLoc);
				findVicNum++;
				logger.info(" * find BLUE victim at: (" + blueLoc.x + "," + blueLoc.y + ")");
			} else if (scoutLoc.x == greenLoc.x && scoutLoc.y == greenLoc.y) {
				model.add(P_GREEN, greenLoc);
				findVicNum++;
				logger.info(" * find GREEN victim at: (" + greenLoc.x + "," + greenLoc.y + ")");
			} else {
				model.add(P_FAKE, scoutLoc);
				logger.info(" * no victim at: (" + scoutLoc.x + "," + scoutLoc.y + ")");
			}

			for (int i = 0; i < psbVics.size(); i++) {
				if (psbVics.get(i).x == scoutLoc.x && psbVics.get(i).y == scoutLoc.y) {
					psbVics.remove(i);
				}
			}

		}

		/** Choose nearest location from all possible victims to go */
		public void pickNextVic() throws Exception {
			// pick nearest victim from all possible victims
			int smallestDis = 99;
			for (int i = 0; i < psbVics.size(); i++) {
				Location currVic = psbVics.get(i);
				if (Math.abs(currVic.x - scoutLoc.x) + Math.abs(currVic.y - scoutLoc.y) < smallestDis) {
					nearestVicLoc = currVic;
					smallestDis = Math.abs(currVic.x - scoutLoc.x) + Math.abs(currVic.y - scoutLoc.y);

				}
			}
		}

		/** Assign optimal path to stack */
		public void findNearestRoute(Location victim) {

			ArrayList<Integer[]> avaiSet = new ArrayList<>();

			// if a location has not been occupied (available), add it to available set
			for (int wid = 0; wid < G_WID; wid++) {
				for (int len = 0; len < G_LEN; len++) {
					if (!model.hasObject(OBS, wid, len)) {
						// a location and its distance to target
						Integer[] Pos = new Integer[3];
						// x location
						Pos[0] = wid;
						// y location
						Pos[1] = len;
						// weight of distance to target
						Pos[2] = Math.abs(victim.x - wid) + Math.abs(victim.y - len);
						avaiSet.add(Pos);

					}
				}
			}

			// use A* search to find optimal route from available set
			Integer[][][] aStarMap = AStarSearch(victim, avaiSet);
			int x = nearestVicLoc.x;
			int y = nearestVicLoc.y;
			Location agLoc = scoutLoc;

			while (x != agLoc.x || y != agLoc.y) {
				Location a = new Location(x, y);
				int f_X = x;
				int f_Y = y;
				x = aStarMap[f_X][f_Y][0];
				y = aStarMap[f_X][f_Y][1];
				nearestVicRoute.push(a);
			}

		}

		/**
		 * search for optimal route by applying A*
		 * 
		 * @param targetX
		 *            x coordinate of target
		 * @param targetY
		 *            y coordinate of target
		 * @param avaiSet
		 *            a set contains all grid that assumed not to be occupied
		 * @return a list of coordinate of all grid in path
		 */
		private Integer[][][] AStarSearch(Location victim, ArrayList<Integer[]> avaiSet) {

			// the 'virtual' current position when searching
			Integer[] virtGrid = new Integer[6];
			Integer[][][] virtGridMtrx = new Integer[G_WID][G_LEN][6];
			Integer[][][] aStarMap = new Integer[G_WID][G_LEN][2];

			// initial A* map
			for (int wid = 0; wid < G_WID; wid++) {
				for (int len = 0; len < G_LEN; len++) {
					virtGridMtrx[wid][len][0] = 99;
					virtGridMtrx[wid][len][1] = 99;
					virtGridMtrx[wid][len][2] = 99;
					virtGridMtrx[wid][len][3] = Math.abs(victim.x - wid) + Math.abs(victim.y - len);
					virtGridMtrx[wid][len][4] = 0;
					virtGridMtrx[wid][len][5] = 0;
				}
			}

			// at beginning it is origin from itself
			virtGrid[0] = model.getAgPos(1).x;
			virtGrid[1] = model.getAgPos(1).y;
			// it is from start point to start point
			virtGrid[2] = 0;
			// it is Manhattan distance from start point to target point
			virtGrid[3] = Math.abs(victim.x - virtGrid[0]) + Math.abs(victim.y - virtGrid[1]);
			// 0 not in openList, 1 in openList
			virtGrid[4] = 1;
			// 0 not in closeList, 1 in closeList
			virtGrid[5] = 0;

			int virtX = virtGrid[0];
			int virtY = virtGrid[1];
			virtGridMtrx[virtX][virtY] = virtGrid;

			// cameFrom contains a two dimension-list of virtPos

			Boolean isOpenListEmpty = false;

			// there is still grid not be evaluated
			while (!isOpenListEmpty) {

				int wid = 0;
				int len = 0;
				int token = 99;
				// find grid which has least a* distance to evaluate
				for (wid = 0; wid < G_WID; wid++) {
					for (len = 0; len < G_LEN; len++)
						if (virtGridMtrx[wid][len][4] == 1 && virtGridMtrx[wid][len][3] < token) {
							// record the next evaluating grid position
							virtGrid = virtGridMtrx[wid][len];
							virtX = wid;
							virtY = len;
							token = virtGrid[3].intValue();
						}
				}

				virtGridMtrx[virtX][virtY][4] = 0;
				virtGridMtrx[virtX][virtY][5] = 1;

				// current position is not destination
				// find neighbors of current position and put undiscovered grids into openSet
				for (Integer[] j : avaiSet) {
					if ((j[0] == virtX + 1 && j[1] == virtY) || (j[0] == virtX - 1 && j[1] == virtY)
							|| (j[0] == virtX && j[1] == virtY + 1) || (j[0] == virtX && j[1] == virtY - 1)) {

						// check if neighbor in openSet or closeSet
						if (virtGridMtrx[j[0]][j[1]][4] == 0 && virtGridMtrx[j[0]][j[1]][5] == 0) {
							virtGridMtrx[j[0]][j[1]][4] = 1;
						}

						// the distance from starting point to neighbor passing current grid equals to
						// distance from starting point to current grid + 1
						int distanceFromStartpoint = virtGrid[2] + 1;

						// update current grid if it is a better route
						if (!(distanceFromStartpoint > virtGridMtrx[j[0]][j[1]][2])) {
							virtGridMtrx[j[0]][j[1]][0] = virtX;
							virtGridMtrx[j[0]][j[1]][1] = virtY;
							virtGridMtrx[j[0]][j[1]][2] = distanceFromStartpoint + virtGridMtrx[j[0]][j[1]][3];
						}
					}
				}

				isOpenListEmpty = true;
				for (wid = 0; wid < G_WID; wid++) {
					for (len = 0; len < G_LEN; len++) {
						aStarMap[wid][len][0] = virtGridMtrx[wid][len][0];
						aStarMap[wid][len][1] = virtGridMtrx[wid][len][1];
						if (virtGridMtrx[wid][len][4] == 1) {
							isOpenListEmpty = false;
						}
					}
				}
			}
			return aStarMap;
		}

		/** Delete robot and all left victim locations if have found all victims */
		public void deleteScout() throws Exception {
			model.remove(AGENT, model.getAgPos(1));
			for (Location l : psbVics) {
				model.remove(P_PSB, l);
			}
		}
	}

	class RobotView extends GridWorldView {

		public RobotView(RobotModel model) {
			super(model, "Mars World", 600);
			defaultFont = new Font("Arial", Font.BOLD, 30); // change default font
			setVisible(true);
			repaint();
		}

		/** draw application objects */
		@Override
		public void draw(Graphics g, int x, int y, int object) {
			switch (object) {
			case RobotEnv.OBS:
				drawGarb(g, x, y);
				break;

			case RobotEnv.TOKEN:
				if (!found)
					drawToken(g, x, y);
				break;

			case RobotEnv.P_PSB:
			case RobotEnv.P_RED:
			case RobotEnv.P_BLUE:
			case RobotEnv.P_GREEN:
			case RobotEnv.P_FAKE:
				drawVic(g, x, y);
				break;

			default:
				break;
			}
		}

		@Override
		public void drawAgent(Graphics g, int x, int y, Color c, int id) {
			String label = "R";
			c = Color.orange;
			if (id == 0) {
				c = Color.darkGray;
			}
			super.drawAgent(g, x, y, c, -1);
			if (id == 0) {
				g.setColor(Color.darkGray);
			} else {
				g.setColor(Color.white);
			}
			super.drawString(g, x, y, defaultFont, label);
		}

		public void drawGarb(Graphics g, int x, int y) {
			super.drawObstacle(g, x, y);
			g.setColor(Color.white);
			drawString(g, x, y, defaultFont, "G");
		}

		public void drawToken(Graphics g, int x, int y) {
			g.setColor(Color.white);
			super.drawEmpty(g, x, y);
			g.setColor(Color.orange);
			super.drawString(g, x, y, defaultFont, "R");
		}

		public void drawVic(Graphics g, int x, int y) {
			g.setColor(Color.white);
			super.drawEmpty(g, x, y);
			if (model.hasObject(P_PSB, x, y)) {
				g.setColor(Color.lightGray);
			} else if (model.hasObject(P_RED, x, y)) {
				g.setColor(Color.red);
			} else if (model.hasObject(P_BLUE, x, y)) {
				g.setColor(Color.blue);
			} else if (model.hasObject(P_GREEN, x, y)) {
				g.setColor(Color.green);
			} else if (model.hasObject(P_FAKE, x, y)) {
				g.setColor(Color.white);
			}
			super.drawString(g, x, y, defaultFont, "V");
		}
	}

	// simulate robot sensor input
	class Simulation {

		private ArrayList<Boolean[]> sr = new ArrayList<Boolean[]>();

		public ArrayList<Boolean[]> returnSimu() {

			Boolean[] s0 = { false, false, false, true };
			Boolean[] s1 = { false, false, false, false };
			Boolean[] s2 = { false, false, true, false };
			Boolean[] s3 = { true, false, true, false };
			Boolean[] s4 = { false, false, true, false };
			Boolean[] s5 = { false, false, false, false };

			sr.add(s0);
			sr.add(s1);
			sr.add(s2);
			sr.add(s3);
			sr.add(s4);
			sr.add(s5);
			return sr;
		}

	}
}
