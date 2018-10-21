
//Authur: HCH
import java.util.ArrayList;
import java.util.HashMap;

import lejos.hardware.lcd.LCD;

public class Env {
	public static final int GSize = 6; // grid size 6*6
	public static final int DirectionSize = 4; // 4 directions

	// boolean for the 6*6 grids (36 grids in total) and with 4 directions
	static boolean[][] record = new boolean[GSize * GSize][DirectionSize];
	static boolean[][][] DirectionRecord = new boolean[GSize * GSize][DirectionSize][DirectionSize];
	boolean[][] visited = new boolean[GSize * GSize][DirectionSize];
	static boolean[][] PossibleLocation = new boolean[GSize * GSize][DirectionSize];

	// two arraylist for storing the path and visited positions
	static ArrayList<Position> path = new ArrayList<Position>();
	static ArrayList<Position> visit = new ArrayList<Position>();
	static ArrayList<Integer> Location = new ArrayList<Integer>();
	static ArrayList<Integer> NextLocation = new ArrayList<Integer>();
	static ArrayList<Integer> Directions = new ArrayList<Integer>();

	static boolean[] startPos = new boolean[GSize * GSize];
	static Position startPosition = null;
	static Position[] curPosition = new Position[GSize * GSize];

	// cur[] shows the boolean for each grid whether there are obstacles in 4
	// directions (scan for only 1-grid distance to secure accuracy)
	static CellType map[][] = new CellType[GSize][GSize];
	static boolean cur[] = new boolean[DirectionSize];
	static int possibleNumber = GSize * GSize;
	static int obstaclePositions[] = { 0, 10, 11, 19, 26, 28 };
	static int victimPositions[] = { 8, 15, 20, 30, 34 };
	HashMap<Distance, ArrayList<Position>> distancePathHashMap = new HashMap<>();

	public static void initMap() {
		possibleNumber -= obstaclePositions.length;
		for (int i = 0; i < GSize; i++) {
			for (int j = 0; j < GSize; j++) {
				map[i][j] = CellType.GridCell;
				int index = j * 6 + i;
				curPosition[index] = new Position(i, j);
			}
		}

		for (int i = 0; i < victimPositions.length; i++) {
			int row = (int) Math.ceil(victimPositions[i] / GSize);
			int col = victimPositions[i] % GSize;
			map[col][row] = CellType.VictimCell;
		}

		for (int i = 0; i < obstaclePositions.length; i++) {
			int row = (int) Math.ceil(obstaclePositions[i] / GSize);
			int col = obstaclePositions[i] % GSize;
			startPos[obstaclePositions[i]] = false;
			map[col][row] = CellType.ObstacleCell;
		}

	}

	// calculating the four boolean for each grid (top,left,right,bottom), totoally
	// 36 grid with each own 4 boolean array
	static void initStartPos() {
		for (int i = 0; i < startPos.length; i++) {
			int row = (int) Math.ceil(i / GSize);
			int col = i % GSize;
			// if top is obstacle
			if (row + 1 >= GSize || map[col][row + 1] == CellType.ObstacleCell) {
				record[i][0] = false;
			} else {
				record[i][0] = true;
			}
			// if left is obstacle
			if (col - 1 < 0 || map[col - 1][row] == CellType.ObstacleCell) {
				record[i][3] = false;
			} else {
				record[i][3] = true;
			}
			// if right is obstacle
			if (col + 1 >= GSize || map[col + 1][row] == CellType.ObstacleCell) {
				record[i][1] = false;
			} else {
				record[i][1] = true;
			}
			// if bottom is obstacle
			if (row - 1 < 0 || map[col][row - 1] == CellType.ObstacleCell) {
				record[i][2] = false;
			} else {
				record[i][2] = true;
			}
			// writing 4 directions for different boolean records(clockwise)
			DirectionRecord[i][0][0] = record[i][0];
			DirectionRecord[i][1][0] = record[i][1];
			DirectionRecord[i][2][0] = record[i][2];
			DirectionRecord[i][3][0] = record[i][3];

			DirectionRecord[i][0][1] = record[i][1];
			DirectionRecord[i][1][1] = record[i][2];
			DirectionRecord[i][2][1] = record[i][3];
			DirectionRecord[i][3][1] = record[i][0];

			DirectionRecord[i][0][2] = record[i][2];
			DirectionRecord[i][1][2] = record[i][3];
			DirectionRecord[i][2][2] = record[i][0];
			DirectionRecord[i][3][2] = record[i][1];

			DirectionRecord[i][0][3] = record[i][3];
			DirectionRecord[i][1][3] = record[i][0];
			DirectionRecord[i][2][3] = record[i][1];
			DirectionRecord[i][3][3] = record[i][2];
		}

	}

	static void location() {
		// scanning 4 directions
		for (int i = 0; i < 4; i++) {
			if (scan(i) == CellType.ObstacleCell) {
				cur[i] = false;
			} else {
				cur[i] = true;
			}
		}

		// checkCur();
		// if (possibleNumber == 1) {
		// for (int i = 0; i < startPos.length; i++) {
		// if (startPos[i]) {
		// int row = (int) Math.ceil(i / GSize);
		// int col;
		// if ( i % GSize == 0) col =6;
		// else col = i%GSize;
		// startPosition = new Position(row, col);
		// LCD.drawString("Position"+ curPosition[i].x + curPosition[i].y , 10, 10);
		// }
		// }
		//
		// } else {
		// try {
		// nextStep();
		// } catch (Exception e) {
		// // TODO Auto-generated catch block
		// e.printStackTrace();
		// }
		// location();
		// }
	}

	static void nextStep() {

		// the movement method for the robot part
		// Position parentPos = null;
		// if (path.size() >= 2) {
		// parentPos = path.get(path.size() - 2);
		// }
		// Position lastPosition = path.get(path.size() - 1);
		// Position position1 = new Position(lastPosition.x + 1, lastPosition.y);//move
		// right
		// Position position2 = new Position(lastPosition.x, lastPosition.y - 1);//move
		// bottom
		// Position position3 = new Position(lastPosition.x, lastPosition.y + 1);//move
		// top
		// Position position4 = new Position(lastPosition.x - 1, lastPosition.y);//move
		// left
		boolean flag = false;
		for (int i = 0; i < DirectionSize; i++) {
			// try to go top first
			if (cur[i] == true) {
				// flag = true;
				if (i == 0) {
					moveUp();
					// visit.add(position3);
					// path.add(position3);
					break;
					// then try to go right
				} else if (i == 1) {
					moveRight();
					// visit.add(position1);
					// path.add(position1);
					break;
					// then go bottom
				} else if (i == 2) {
					moveDown();
					// visit.add(position2);
					// path.add(position2);
					break;
					// finally go left
				} else if (i == 3) {
					moveLeft();
					// visit.add(position4);
					// path.add(position4);
					break;
				}
			}
		}
		// if (!flag) {
		// if (path.size() == 0) {
		// throw new Exception("location error");
		// }
		// path.remove(lastPosition);
		// nextStep();
		// }
	}

	// static void checkCur() {
	// for (int i = 0; i < GSize * GSize; i++) {
	// if (!startPos[i])
	// continue;
	//
	// // set flag false if the position is not possible again.
	// boolean flag = true;
	// Position lastMovement = path.get(path.size() - 1);
	// curPosition[i].x += lastMovement.x;
	// curPosition[i].y += lastMovement.y;
	// if (curPosition[i].isInvalid()) {
	// flag = false;
	// } else {
	// int index = (curPosition[i].y * GSize) -1 + curPosition[i].x;
	// for (int j = 0; j < DirectionSize; j++) {
	// for(int a = 0; a < DirectionSize; a++){
	// if (cur[j] != DirectionRecord[index][a][j]) {
	// flag = false;
	// PossibleLocation[index][j] = false;
	// possibleNumber--;
	// break;
	// }
	// }
	// }
	// }
	// if (!flag) {
	// possibleNumber--;
	// }
	// startPos[i] = flag;
	// }
	// }
	static CellType scan(int j) {
		// if there is an obstacle in front
		if (j == 0 && PilotRobot.frontDistance <= 20) {
			return CellType.ObstacleCell;
		}
		if (j == 0 && PilotRobot.frontDistance >= 20) {
			return CellType.GridCell;
		}
		// if there is an obstacle in the right
		if (j == 1 && PilotRobot.rightDistance <= 20) {
			return CellType.ObstacleCell;
		}
		if (j == 1 && PilotRobot.frontDistance >= 20) {
			return CellType.GridCell;
		}
		// if there is an obstacle on the bottom
		if (j == 2 && PilotRobot.behindDistance <= 20) {
			return CellType.ObstacleCell;
		}
		if (j == 2 && PilotRobot.frontDistance >= 20) {
			return CellType.GridCell;
		}
		// if there is an obstacle on the left
		if (j == 3 && PilotRobot.leftDistance <= 20) {
			return CellType.ObstacleCell;
		}
		if (j == 3 && PilotRobot.frontDistance >= 20) {
			return CellType.GridCell;
		} else {
			return CellType.GridCell;
		}
	}

	/*
	 * Move method for the robot, which will also keep its heading to the original
	 * heading when u put it into arena in case it affect the scanning results
	 */
	static void moveDown() {
		PilotRobot.rotateRight();
		PilotRobot.rotateRight();
		PilotRobot.moveForward();
		PilotRobot.rotateRight();
		PilotRobot.rotateRight();
	}

	static void moveLeft() {
		PilotRobot.rotateLeft();
		PilotRobot.moveForward();
		PilotRobot.rotateRight();
	}

	static void moveRight() {
		PilotRobot.rotateRight();
		PilotRobot.moveForward();
		PilotRobot.rotateLeft();
	}

	static void moveUp() {
		PilotRobot.moveForward();
	}

	/*
	 * The first matching check (Becasue the number of starting position was 36 and
	 * later stored into an array list so I used two different method for checking
	 * it)
	 */
	static void FirstCheck() {
		// for each grid
		for (int index = 0; index < GSize * GSize; index++) {
			// then for each direction for the known grids
			for (int j = 0; j < DirectionSize; j++) {
				// finally 4 scanning results
				for (int a = 0; a < DirectionSize; a++) {
					// if it matches, add the location to the possible locations
					if (cur[a] == DirectionRecord[index][a][j]) {
						PossibleLocation[index][j] = true;
						Location.add(index);
						// The heading should also be stored
						Directions.add(j);
						int x = index % 6;
						// Still need to convet the numebr to positions :(
						int y = (int) Math.ceil(index / 6);
						if (j == 0) {
							// if it is facing north (The usual facing)
							// first going top
							if (DirectionRecord[index][0][j] = true) {
								y++;
								// if top is blocked then try to get right
							} else if (DirectionRecord[index][1][j] = true) {
								x++;
								// if top and right is blocked try to go down
							} else if (DirectionRecord[index][2][j] = true) {
								y--;
								// finally try to go left
							} else if (DirectionRecord[index][3][j] = true) {
								x--;
							}
							int coord = y * 6 + x;
							NextLocation.add(coord);
						} // if it is facing east
						if (j == 1) {
							// going "top" is going right actually
							if (DirectionRecord[index][0][j] == true) {
								x++;
								// the same for going "right"Bottom
							} else if (DirectionRecord[index][1][j] == true) {
								y--;
								// the same for going "Bottom"Left
							} else if (DirectionRecord[index][2][j] == true) {
								x--;
								// the same for going "Left" top
							} else if (DirectionRecord[index][3][j] == true) {
								y++;
							}
							int coord = y * 6 + x;
							NextLocation.add(coord);
						}
						// same as top
						if (j == 2) {
							if (DirectionRecord[index][0][j] == true) {
								y--;
							} else if (DirectionRecord[index][1][j] == true) {
								x--;
							} else if (DirectionRecord[index][2][j] == true) {
								y++;
							} else if (DirectionRecord[index][3][j] == true) {
								x++;
							}
							int coord = y * 6 + x;
							NextLocation.add(coord);
						}
						// same as top
						if (j == 3) {
							if (DirectionRecord[index][0][j] == true) {
								x--;
							} else if (DirectionRecord[index][1][j] == true) {
								y++;
							} else if (DirectionRecord[index][2][j] == true) {
								x++;
							} else if (DirectionRecord[index][3][j] == true) {
								y--;
							}
							int coord = y * 6 + x;
							NextLocation.add(coord);
						}

					} else {
						PossibleLocation[index][j] = false;
					}

				}
			}
		}
	}

	static void RecurCheck() {
		// do this recursively until there is only one determined position
		while (Location.size() != 1) {
			// a new array list for storing next location cuz next location will be cleared
			// for storing new result
			ArrayList<Integer> Temp = NextLocation;
			NextLocation.clear();
			// also need to clear the locations because by now there is definitly not one
			// results
			Location.clear();
			Directions.clear();
			// Tell the robot to move
			nextStep();
			// need to do one more scan
			PilotRobot.scanneAround();
			// updating the cur[i] for next matching
			location();
			for (int i = 0; i < Temp.size(); i++) {
				for (int j = 0; j < DirectionSize; j++) {
					for (int a = 0; a < DirectionSize; a++) {
						// matching the results from the previous matching (which by somehow is deleting
						// beliefs)
						if (cur[a] == DirectionRecord[Temp.get(i)][a][j]) {
							PossibleLocation[Temp.get(i)][j] = true;
							// updating the Location and Directions
							Location.add(Temp.get(i));
							Directions.add(j);
							int x = Temp.get(i) % 6;
							int y = (int) Math.ceil(Temp.get(i) / 6);
							if (j == 0) {
								if (DirectionRecord[Temp.get(i)][0][j] == true) {
									y++;
								} else if (DirectionRecord[Temp.get(i)][1][j] == true) {
									x++;
								} else if (DirectionRecord[Temp.get(i)][2][j] == true) {
									y--;
								} else if (DirectionRecord[Temp.get(i)][3][j] == true) {
									x--;
								}
								int index = y * 6 + x;
								NextLocation.add(index);
							}
							if (j == 1) {
								if (DirectionRecord[Temp.get(i)][0][j] == true) {
									x++;
								} else if (DirectionRecord[Temp.get(i)][1][j] == true) {
									y--;
								} else if (DirectionRecord[Temp.get(i)][2][j] == true) {
									x--;
								} else if (DirectionRecord[Temp.get(i)][3][j] == true) {
									y++;
								}
								int index = y * 6 + x;
								NextLocation.add(index);
							}
							if (j == 2) {
								if (DirectionRecord[Temp.get(i)][0][j] == true) {
									y--;
								} else if (DirectionRecord[Temp.get(i)][1][j] == true) {
									x--;
								} else if (DirectionRecord[Temp.get(i)][2][j] == true) {
									y++;
								} else if (DirectionRecord[Temp.get(i)][3][j] == true) {
									x++;
								}
								int index = y * 6 + x;
								NextLocation.add(index);
							}
							if (j == 3) {
								if (DirectionRecord[Temp.get(i)][0][j] == true) {
									x--;
								} else if (DirectionRecord[Temp.get(i)][1][j] == true) {
									y++;
								} else if (DirectionRecord[Temp.get(i)][2][j] == true) {
									x++;
								} else if (DirectionRecord[Temp.get(i)][3][j] == true) {
									y--;
								}
								int index = y * 6 + x;
								NextLocation.add(index);
								// System.out.print(Location);
								// LCD.clear();
							}
						}

					}
				}
			}

		} // once there is only one position left in the Location
			// Calculating the corresponding position :(

		MainRobot.curr.x = Location.get(0) % 6;

		MainRobot.curr.y = (int) Math.ceil(Location.get(0) / 6);
		System.out.print(MainRobot.curr.x + " " + MainRobot.curr.y);
		// also need to update the facing angle in the A* search
		if (Directions.get(0) == 0) {
			MainRobot.currAngle = 0;
		}
		if (Directions.get(0) == 1) {
			MainRobot.currAngle = 90;
		}
		if (Directions.get(0) == 2) {
			MainRobot.currAngle = 180;
		}
		if (Directions.get(0) == 3) {
			MainRobot.currAngle = -90;
		}
	}

	//
	//
	//
	//
	//
	// below is my path planning method but used a mixture of depth-first search and
	// A* search so later I decided to use Jonny's A* search
	// if u have time maybe u can take a look and tell me is that good :)
	// Cuz I just wrote it this time not copied from last assignment
	void findOutVictims() {
		int dp[][] = new int[GSize][GSize];
		for (int r = 0; r < GSize; r++) {
			for (int c = 0; c < GSize; c++) {

				dp[r][c] = Integer.MAX_VALUE;

			}
		}
		LinkedList<Position> thequeue = new LinkedList<Position>();
		boolean[] marked = new boolean[GSize * GSize];
		for (int i = 0; i < GSize * GSize; i++) {
			marked[i] = false;
		}

		for (int i = 0; i < victimPositions.length; i++) {
			// solve the smallest distance path
			Position newPosition = new Position(victimPositions[i] / GSize, victimPositions[i] % GSize);
			thequeue.offer(newPosition);
			if (newPosition.isInvalid()) {
				continue;
			}

			marked[victimPositions[i]] = true;
			int depth = 1;
			Position firstPosition = newPosition;
			Position lastPosition = newPosition;

			while (!thequeue.isEmpty()) {
				Position next = (Position) thequeue.poll();
				if (next.equals(lastPosition)) {
					depth++;
					firstPosition = thequeue.peekFirst();
					lastPosition = thequeue.peekLast();
				}
				Position position1 = new Position(next.x + 1, next.y);
				Position position2 = new Position(next.x, next.y - 1);
				Position position3 = new Position(next.x, next.y + 1);
				Position position4 = new Position(next.x - 1, next.y);
				Position[] positions = { position1, position2, position3, position4 };
				for (int j = 0; j < DirectionSize; j++) {
					int index = positions[i].x * GSize + positions[i].y;
					if (marked[index] == false && position1.isInvalid()) {
						thequeue.offer(positions[i]);
						dp[positions[i].x][positions[i].y] = depth;
						marked[index] = true;
					}
				}
			}

			addDistanceRecord(newPosition, dp);
			ArrayList<Position> movePath = generateBestPath();

			for (int in = 1; i < movePath.size(); in++) {
				Position position1 = movePath.get(in - 1);
				Position position2 = movePath.get(in);
				if (map[position2.x][position2.y] == CellType.VictimCell) {
					scanJudge(position2);
				}
				if (position1.x == position2.x) {
					if (position1.y - 1 == position2.y) {
						moveLeft();
					} else {
						moveRight();
					}
				} else {
					if (position1.x - 1 == position2.x) {
						moveUp();
					} else {
						moveDown();
					}
				}
			}
		}
	}

	public void swap(Position[] positions, int i, int j) {
		Position temp = positions[i];
		positions[i] = positions[j];
		positions[j] = temp;
	}

	public ArrayList<Distance> findBestPath(Position[] positions, int st, int len) {
		ArrayList<Distance> result = new ArrayList<Distance>();
		ArrayList<Distance> tempDistances = new ArrayList<Distance>();
		ArrayList<Position> temPositions = new ArrayList<>();
		int min = Integer.MAX_VALUE;
		int sum = 0;
		if (st == len - 1) {
			for (int i = 0; i < len; i++) {
				temPositions.add(positions[i]);
			}
			for (int i = 0; i < temPositions.size(); i++) {
				Distance distance = null;
				Position mPosition = null;
				if (i == 0) {
					mPosition = startPosition;
				} else {
					mPosition = temPositions.get(i - 1);
				}
				distance = new Distance(mPosition, temPositions.get(i), 0);

				for (Distance temp : distancePathHashMap.keySet()) {
					if (temp.equals(distance)) {
						sum += temp.distance;
					}
					tempDistances.add(temp);
				}
			}
			if (sum < min) {
				min = sum;
				result = tempDistances;
			}
			tempDistances = new ArrayList<>();
			temPositions.clear();
		} else {
			for (int i = st; i < len; i++) {
				swap(positions, st, i);
				findBestPath(positions, st + 1, len);
				swap(positions, st, i);
			}
		}
		return result;
	}

	ArrayList<Position> generateBestPath() {
		Position[] positions = new Position[victimPositions.length];
		for (int i = 0; i < victimPositions.length; i++) {
			Position position = new Position(victimPositions[i] / GSize, victimPositions[i] % GSize);
			positions[i] = position;
		}
		ArrayList<Distance> distances = findBestPath(positions, 0, victimPositions.length);
		ArrayList<Position> res = new ArrayList<>();
		for (int i = 0; i < distances.size(); i++) {
			res.addAll(distancePathHashMap.get(distances.get(i)));
		}
		return res;
	}

	void addDistanceRecord(Position newPosition, int[][] dp) {
		// find the path
		int originIndex = newPosition.x * GSize + newPosition.y;
		Distance distance = null;
		int index = startPosition.x * GSize + startPosition.y;
		if (index < originIndex) {
			distance = new Distance(startPosition, newPosition, dp[startPosition.x][startPosition.y]);
		} else {
			distance = new Distance(newPosition, startPosition, dp[startPosition.x][startPosition.y]);
		}
		Position cur1 = startPosition;
		ArrayList<Position> pathArrayList = new ArrayList<Position>();
		pathArrayList.add(cur1);
		while (!cur1.equals(newPosition)) {
			Position[] positions = getAdjacentPositions(cur1);
			for (int m = 0; m < positions.length; m++) {
				if (dp[positions[m].x][positions[m].y] == dp[cur1.x][cur1.y] - 1) {
					pathArrayList.add(positions[m]);
				}
				cur1 = positions[m];
			}
		}

		distancePathHashMap.put(distance, pathArrayList);
		for (int j = 0; j < victimPositions.length; j++) {
			if (originIndex == victimPositions[j])
				continue;
			Position victimPosition = new Position(victimPositions[j] / GSize, victimPositions[j] % GSize);
			if (originIndex < victimPositions[j]) {
				distance = new Distance(newPosition, victimPosition, dp[startPosition.x][startPosition.y]);
			} else {
				distance = new Distance(victimPosition, newPosition, dp[startPosition.x][startPosition.y]);
			}

			Position cur2 = startPosition;
			ArrayList<Position> tempArrayList = new ArrayList<Position>();
			pathArrayList.add(cur1);
			while (!cur1.equals(newPosition)) {
				Position[] positions = getAdjacentPositions(cur1);
				for (int m = 0; m < positions.length; m++) {
					if (dp[positions[m].x][positions[m].y] == dp[cur1.x][cur1.y] - 1) {
						pathArrayList.add(positions[m]);
					}
					cur1 = positions[m];
				}
			}

			if (!distancePathHashMap.containsKey(distance)) {
				distancePathHashMap.put(distance, tempArrayList);
			}
		}
	}

	Position[] getAdjacentPositions(Position position) {
		Position position1 = new Position(position.x + 1, position.y);
		Position position2 = new Position(position.x, position.y - 1);
		Position position3 = new Position(position.x, position.y + 1);
		Position position4 = new Position(position.x - 1, position.y);
		Position[] positions = { position1, position2, position3, position4 };
		return positions;
	}
}