package com.example.android.accelerometerplay;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Random;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.DisplayMetrics;

public class Maze
{
  private float pixelCellSizeX, pixelCellSizeY;
  private int dimensionX, dimensionY; // dimension of maze
  private int gridDimensionX, gridDimensionY; // dimension of output grid
  private char[][] grid; // output grid
  private Cell[][] cells; // 2d array of Cells
  private Random random = new Random(); // The random object
  private ArrayList<RectF> walls = new ArrayList<RectF>();
  
  
  // initialize with x and y the same
  public Maze(int aDimension, DisplayMetrics metrics ) {
      // Initialize
      this(aDimension, aDimension, metrics);
  }
  // constructor
  public Maze(int xDimension, int yDimension, DisplayMetrics metrics) {
	  pixelCellSizeX = (float)metrics.widthPixels  / (float)( xDimension * 4 + 1);
	  pixelCellSizeY = (float)metrics.heightPixels / (float)( yDimension * 4 + 1);
      dimensionX = xDimension;
      dimensionY = yDimension;
      
      gridDimensionX = xDimension * 4 + 1;
      gridDimensionY = yDimension * 4 + 1;
      grid = new char[gridDimensionX][gridDimensionY];
      init();
      generateMaze();
      updateGrid();
  }

  private void init() {
      // create cells
      cells = new Cell[dimensionX][dimensionY];
      for (int x = 0; x < dimensionX; x++) {
          for (int y = 0; y < dimensionY; y++) {
              cells[x][y] = new Cell(x, y, false); // create cell (see Cell constructor)
          }
      }
  }

  // inner class to represent a cell
  private class Cell {
    int x, y; // coordinates
    // cells this cell is connected to
    ArrayList<Cell> neighbors = new ArrayList<Cell>();
    // solver: if already used
    boolean visited = false;
    // solver: the Cell before this one in the path
    Cell parent = null;
    // solver: if used in last attempt to solve path
    boolean inPath = false;
    // solver: distance traveled this far
    double travelled;
    // solver: projected distance to end
    double projectedDist;
    // impassable cell
    boolean wall = true;
    // if true, has yet to be used in generation
    boolean open = true;
    // construct Cell at x, y
    Cell(int x, int y) {
        this(x, y, true);
    }
    // construct Cell at x, y and with whether it isWall
    Cell(int x, int y, boolean isWall) {
        this.x = x;
        this.y = y;
        this.wall = isWall;
    }
    // add a neighbor to this cell, and this cell as a neighbor to the other
    void addNeighbor(Cell other) {
        if (!this.neighbors.contains(other)) { // avoid duplicates
            this.neighbors.add(other);
        }
        if (!other.neighbors.contains(this)) { // avoid duplicates
            other.neighbors.add(this);
        }
    }
    // used in updateGrid()
    boolean isCellBelowNeighbor() {
        return this.neighbors.contains(new Cell(this.x, this.y + 1));
    }
    // used in updateGrid()
    boolean isCellRightNeighbor() {
        return this.neighbors.contains(new Cell(this.x + 1, this.y));
    }
    // useful Cell representation
    @Override
    public String toString() {
        return String.format("Cell(%s, %s)", x, y);
    }
    // useful Cell equivalence
    @Override
    public boolean equals(Object other) {
        if (!(other instanceof Cell)) return false;
        Cell otherCell = (Cell) other;
        return (this.x == otherCell.x && this.y == otherCell.y);
    }
    // should be overridden with equals
    @Override
    public int hashCode() {
        // random hash code method designed to be usually unique
        return this.x + this.y * 256;
    }
  }
  // generate from upper left (In computing the y increases down often)
  private void generateMaze()
  {
      generateMaze(0, 0);
  }
  
  private int cellStartX, cellStartY;
  
  public RectF getStartZone()
  {
	  RectF zone = new RectF();
	  zone.left = ( cellStartX * 4 + 1) * pixelCellSizeX;
	  zone.top  = ( cellStartY * 4 + 1) * pixelCellSizeY;
	  zone.bottom = zone.top + pixelCellSizeY * 3.0f;
	  zone.right  = zone.left + pixelCellSizeX * 3.0f;
	  return zone;
  }
  
  public RectF getEndZone()
  {
	  RectF zone = new RectF();
	  zone.left = ( ( dimensionX - 1 ) * 4 + 1 ) * pixelCellSizeX;
	  zone.top  = ( ( dimensionY - 1 ) * 4 + 1 ) * pixelCellSizeY;
	  zone.bottom = zone.top + pixelCellSizeY * 3.0f;
	  zone.right  = zone.left + pixelCellSizeX * 3.0f;
	  return zone;
  }
  
  // generate the maze from coordinates x, y
  private void generateMaze(int x, int y)
  {
	  cellStartX = x;
	  cellStartY = y;
      generateMaze(getCell(x, y)); // generate from Cell
  }
  
  public void setStart( Particle ball )
  {
	  ball.setPos(2.0f * pixelCellSizeX, 2.0f * pixelCellSizeY);
  }
  
  public void checkWalls( Particle ball, Canvas canvas )
  {
	  float x, y, d, r, lastX, lastY, newX, newY;
	  x = ball.getPosX();
	  y = ball.getPosY();
	  lastX = ball.getLastPosX();
	  lastY = ball.getLastPosY();
	  d = ball.getDiameter();
	  r = d / 2.0f;
	  
	  RectF objLast = new RectF(lastX - r, lastY - r, lastX + r, lastY + r );
	  
	  for( int i = 0; i < walls.size(); i++ )
	  {
    	  RectF obj = new RectF(x - r, y - r, x + r, y + r );
		  RectF wall = walls.get(i); 		  
		  
		  if( obj.intersect(wall) )
		  {
			  // left or right
			  if( obj.centerX() < objLast.centerX() )
			  {
				  // ball needs to move to the right
				  newX = wall.right + r;
			  }
			  else
			  {
				  newX = wall.left - r;
			  }
			  
			  // top or bottom
			  if( obj.centerY() < objLast.centerY() )
			  {
				  // ball needs to move down
				  newY = wall.bottom + r;
				  if( Math.abs(newY - y) > 20 )
				  {
					  newY = wall.top - r;
				  }
			  }
			  else
			  {
				// ball needs to move up
				  newY = wall.top - r;
				  if( Math.abs(newY - y) > 20 )
				  {
					  newY = wall.bottom + r;
				  }
			  }
			  
			  // TODO: Improve logic of deciding to move up / down vs left/right			  
			  if( obj.top == wall.top || obj.bottom == wall.bottom )
			  {
				  y = newY;
			  }
			  else
			  {
				  x = newX;
			  }
			  
			  ball.setPos(x, y);
			  
			  RectF obj2 = new RectF( x - r, y - r, x + r, y + r);
			  if( obj2.intersect(wall) )
			  {
				  DebugDraw.AddRectF(obj2, Color.BLUE );
			  }
		  }
	  }
  }
  
  public float getCellSizeX()
  {
	  return pixelCellSizeX;
  }
  
  public float getCellSizeY()
  {
	  return pixelCellSizeY;
  }
  
  private void generateMaze(Cell startAt)
  {
      // don't generate from cell not there
      if (startAt == null) return;
      startAt.open = false; // indicate cell closed for generation
      ArrayList<Cell> cells = new ArrayList<Cell>();
      cells.add(startAt);

      while (!cells.isEmpty()) {
          Cell cell;
          // this is to reduce but not completely eliminate the number
          //   of long twisting halls with short easy to detect branches
          //   which results in easy mazes
          if (random.nextInt(10)==0)
              cell = cells.remove(random.nextInt(cells.size()));
          else cell = cells.remove(cells.size() - 1);
          // for collection
          ArrayList<Cell> neighbors = new ArrayList<Cell>();
          // cells that could potentially be neighbors
          Cell[] potentialNeighbors = new Cell[]{
              getCell(cell.x + 1, cell.y),
              getCell(cell.x, cell.y + 1),
              getCell(cell.x - 1, cell.y),
              getCell(cell.x, cell.y - 1)
          };
          for (Cell other : potentialNeighbors) {
              // skip if outside, is a wall or is not opened
              if (other==null || other.wall || !other.open) continue;
              neighbors.add(other);
          }
          if (neighbors.isEmpty()) continue;
          // get random cell
          Cell selected = neighbors.get(random.nextInt(neighbors.size()));
          // add as neighbor
          selected.open = false; // indicate cell closed for generation
          cell.addNeighbor(selected);
          cells.add(cell);
          cells.add(selected);
      }
  }
  // used to get a Cell at x, y; returns null out of bounds
  public Cell getCell(int x, int y)
  {
      try
      {
          return cells[x][y];
      }
      catch (ArrayIndexOutOfBoundsException e)
      { // catch out of bounds
          return null;
      }
  }

  public void solve()
  {
      // default solve top left to bottom right
      this.solve(0, 0, dimensionX - 1, dimensionY -1);
  }
  
  // solve the maze starting from the start state (A-star algorithm)
  public void solve(int startX, int startY, int endX, int endY)
  {
      // re initialize cells for path finding
      for (Cell[] cellrow : this.cells)
      {
          for (Cell cell : cellrow)
          {
              cell.parent = null;
              cell.visited = false;
              cell.inPath = false;
              cell.travelled = 0;
              cell.projectedDist = -1;
          }
      }
      
      // cells still being considered
      ArrayList<Cell> openCells = new ArrayList<Cell>();
      // cell being considered
      Cell endCell = getCell(endX, endY);
      
      if (endCell == null) return; // quit if end out of bounds
      { // anonymous block to delete start, because not used later
          Cell start = getCell(startX, startY);
          if (start == null) return; // quit if start out of bounds
          start.projectedDist = getProjectedDistance(start, 0, endCell);
          start.visited = true;
          openCells.add(start);
      }
      
      boolean solving = true;
      while (solving)
      {
          if (openCells.isEmpty()) return; // quit, no path
      
          // sort openCells according to least projected distance
          Collections.sort(openCells, new Comparator<Cell>(){
              @Override
              public int compare(Cell cell1, Cell cell2) {
                  double diff = cell1.projectedDist - cell2.projectedDist;
                  if (diff > 0) return 1;
                  else if (diff < 0) return -1;
                  else return 0;
              }
          });
          
          Cell current = openCells.remove(0); // pop cell least projectedDist
          if (current == endCell) break; // at end
          for (Cell neighbor : current.neighbors) {
              double projDist = getProjectedDistance(neighbor,
                      current.travelled + 1, endCell);
              if (!neighbor.visited || // not visited yet
                      projDist < neighbor.projectedDist) { // better path
                  neighbor.parent = current;
                  neighbor.visited = true;
                  neighbor.projectedDist = projDist;
                  neighbor.travelled = current.travelled + 1;
                  if (!openCells.contains(neighbor))
                      openCells.add(neighbor);
              }
          }
      }
      // create path from end to beginning
      Cell backtracking = endCell;
      backtracking.inPath = true;
      while (backtracking.parent != null) {
          backtracking = backtracking.parent;
          backtracking.inPath = true;
      }
  }
  // get the projected distance
  // (A star algorithm consistent)
  public double getProjectedDistance(Cell current, double travelled, Cell end)
  {
      return travelled + Math.abs(current.x - end.x) + 
              Math.abs(current.y - current.x);
  }
  
  // draw the maze
  public void updateGrid()
  {
      char backChar = ' ', wallChar = '+', cellChar = ' ', pathChar = '*';
      // fill background
      for (int x = 0; x < gridDimensionX; x ++) {
          for (int y = 0; y < gridDimensionY; y ++) {
              grid[x][y] = backChar;
          }
      }
      // build walls
      for (int x = 0; x < gridDimensionX; x ++) {
          for (int y = 0; y < gridDimensionY; y ++) {
              if (x % 4 == 0 || y % 4 == 0)
                  grid[x][y] = wallChar;
          }
      }
      
      // make meaningful representation
      for (int x = 0; x < dimensionX; x++)
      {
          for (int y = 0; y < dimensionY; y++)
          {
              Cell current = getCell(x, y);
              int gridX = x * 4 + 2, gridY = y * 4 + 2;
              if (current.inPath) {
                  grid[gridX][gridY] = pathChar;
                  if (current.isCellBelowNeighbor())
                      if (getCell(x, y + 1).inPath)
                      {
                    	  grid[gridX - 1][gridY + 1] = backChar;
                    	  grid[gridX][gridY + 1] = pathChar;
                    	  grid[gridX + 1][gridY + 1] = backChar;
                    	  
                    	  grid[gridX - 1][gridY + 2] = backChar;
                    	  grid[gridX][gridY + 2] = pathChar;
                    	  grid[gridX + 1][gridY + 2] = backChar;
                    	  
                    	  grid[gridX - 1][gridY + 3] = backChar;
                    	  grid[gridX][gridY + 3] = pathChar;
                    	  grid[gridX + 1][gridY + 3] = backChar;                   
                      }
                      else
                      {
                    	  grid[gridX - 1][gridY + 1] = backChar;
                    	  grid[gridX][gridY + 1] = cellChar;
                    	  grid[gridX + 1][gridY + 1] = backChar;
                    	  
                    	  grid[gridX - 1][gridY + 2] = backChar;
                    	  grid[gridX][gridY + 2] = cellChar;
                    	  grid[gridX + 1][gridY + 2] = backChar;
                    	  
                    	  grid[gridX - 1][gridY + 3] = backChar;
                    	  grid[gridX][gridY + 3] = cellChar;
                    	  grid[gridX + 1][gridY + 3] = backChar;  
                      }
                  if (current.isCellRightNeighbor())
                      if (getCell(x + 1, y).inPath)
                      {
                    	  grid[gridX + 1][gridY - 1] = backChar;
                    	  grid[gridX + 1][gridY]     = pathChar;
                          grid[gridX + 1][gridY + 1] = backChar;
                    	  
                    	  grid[gridX + 2][gridY - 1] = backChar;
                    	  grid[gridX + 2][gridY]     = pathChar;
                          grid[gridX + 2][gridY + 1] = backChar;
                          
                    	  grid[gridX + 3][gridY - 1] = backChar;
                    	  grid[gridX + 3][gridY]     = pathChar;
                          grid[gridX + 3][gridY + 1] = backChar;
                      } else
                      {
                    	  grid[gridX + 1][gridY - 1] = backChar;
                    	  grid[gridX + 1][gridY]     = cellChar;
                          grid[gridX + 1][gridY + 1] = backChar;
                    	  
                    	  grid[gridX + 2][gridY - 1] = backChar;
                    	  grid[gridX + 2][gridY]     = cellChar;
                          grid[gridX + 2][gridY + 1] = backChar;
                          
                    	  grid[gridX + 3][gridY - 1] = backChar;
                    	  grid[gridX + 3][gridY]     = cellChar;
                          grid[gridX + 3][gridY + 1] = backChar;
                      }
              }
              else
              {
                  grid[gridX][gridY] = cellChar;
                  if (current.isCellBelowNeighbor())
                  {
                      grid[gridX - 1][gridY + 1] = backChar;
                      grid[gridX][gridY + 1] = cellChar;
                      grid[gridX + 1][gridY + 1] = backChar;

                      grid[gridX - 1][gridY + 2] = backChar;
                      grid[gridX][gridY + 2] = cellChar;
                      grid[gridX + 1][gridY + 2] = backChar;

                      grid[gridX - 1][gridY + 3] = backChar;
                      grid[gridX][gridY + 3] = cellChar;
                      grid[gridX + 1][gridY + 3] = backChar;

                  }
                  if (current.isCellRightNeighbor())
                  {
                      grid[gridX + 1][gridY - 1] = backChar;
                      grid[gridX + 1][gridY] = cellChar;
                      grid[gridX + 1][gridY + 1] = backChar;
                      
                      grid[gridX + 2][gridY - 1] = backChar;
                      grid[gridX + 2][gridY] = cellChar;
                      grid[gridX + 2][gridY + 1] = backChar;
                      
                      grid[gridX + 3][gridY - 1] = backChar;
                      grid[gridX + 3][gridY] = cellChar;
                      grid[gridX + 3][gridY + 1] = backChar;
                  }
              }
          }
      }
      
      // Make collidable walls
      boolean wallUsed[][][] = new boolean[gridDimensionX][gridDimensionY][2];
      
      for( int i= 0; i < gridDimensionY; i++ )
      {
    	  for( int j= 0; j < gridDimensionX; j++ )
    	  {
    		  wallUsed[j][i][0] = false;
    		  wallUsed[j][i][1] = false;
    	  }
      }
      
      walls.clear();
      // for each row..
      for( int i = 0; i < gridDimensionY; i++ )
      {
    	  for( int j = 0; j < gridDimensionX; j++ )
    	  {
    		  if( j < gridDimensionX - 1 && !wallUsed[j][i][0] && grid[j][i] == '+' && grid[j + 1][i] == '+' && !wallUsed[j+1][i][0] )
    		  {
    			  // horizontal wall
    			  wallUsed[j][i][0]   = true;
    			  wallUsed[j+1][i][0] = true;
    			  int k;
    			  for( k = j + 2; k < gridDimensionX && grid[k][i] == '+' && !wallUsed[k][i][0]; k++ )
    			  {
    				  wallUsed[k][i][0] = true;
    			  }
    			  
    		      RectF wall = new RectF();
    		      wall.left = ( (float)j /*+ 0.5f*/ ) * pixelCellSizeX;
    		      wall.top = (float)i * pixelCellSizeY;
    		      wall.bottom = wall.top + pixelCellSizeY;
    		      wall.right = ( (float)k/* - 0.5f */) * pixelCellSizeX;
    		      walls.add(wall);
    		  } 
    		  else if ( i < gridDimensionY - 1 && !wallUsed[j][i][1] && grid[j][i] == '+' && grid[j][i+1] == '+' && !wallUsed[j][i+1][1] )
    		  {
    			  // vertical wall
    			  wallUsed[j][i][1]   = true;
    			  wallUsed[j][i+1][1] = true;
    			  int k;
    			  for( k = i + 2; k < gridDimensionY && grid[j][k] == '+' && !wallUsed[j][k][1]; k++ )
    			  {
    				  wallUsed[j][k][1] = true;
    			  }
    			  
    		      RectF wall = new RectF();
    		      wall.left = (float)j * pixelCellSizeX;
    		      wall.top = ( (float)i /*+ 0.5f*/ ) * pixelCellSizeY;
    		      wall.bottom = ( (float)k/* - 0.5f*/ ) * pixelCellSizeY;
    		      wall.right = wall.left + pixelCellSizeX;
    		      walls.add(wall);
    		  }
    	  }
      }
  }
  
  public void drawWalls( Canvas canvas )
  {
      if( canvas != null )
      {
    	  Paint paint = new Paint();
    	  paint.setColor( Color.WHITE );
    	  for( int i = 0; i < walls.size(); i++ )
    	  {
    		  canvas.drawRect( walls.get(i), paint);
    	  }
      }
  }
  
  // forms a meaningful representation
  @Override
  public String toString()
  {
      updateGrid();
      String output = "";
      for (int y = 0; y < gridDimensionY; y++) {
          for (int x = 0; x < gridDimensionX; x++) {
              output += grid[x][y];
          }
          output += "\n";
      }
      return output;
  }
}
