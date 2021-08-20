function collision = SegmentRectangleCollision(seg, rect)
  collision = IsPointInsideRectangle([seg(1), seg(2)], rect) || ...
              IsPointInsideRectangle([seg(3), seg(4)], rect) || ...
              SegmentsIntersect(seg(1), seg(2), seg(3), seg(4), rect(1), rect(2), rect(3), rect(2)) || ...
              SegmentsIntersect(seg(1), seg(2), seg(3), seg(4), rect(1), rect(4), rect(3), rect(4)) || ...
              SegmentsIntersect(seg(1), seg(2), seg(3), seg(4), rect(1), rect(2), rect(1), rect(4)) || ...
              SegmentsIntersect(seg(1), seg(2), seg(3), seg(4), rect(3), rect(2), rect(3), rect(4)); 
end


function res = IsPointInsideRectangle(p, rect)
res = p(1) >= rect(1) && p(1) <= rect(3) && p(2) >= rect(2) && p(2) <= rect(4);
end


function collision = SegmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4)
%code adapted from GraphicsGem
%http://www.realtimerendering.com/resources/GraphicsGems/gemsiii/insectc.c

        
        Ax = x2 - x1;
        Bx = x3 - x4;

        if (Ax < 0)
                x1lo = x2;
                x1hi = x1;
        else
                x1hi = x2;
                x1lo = x1;
        end
        
        if (Bx > 0)
                if (x1hi < x4 || x3 < x1lo)
                   collision = false;
                   return;   
                end
        else
                if (x1hi < x3 || x4 < x1lo)
                   collision = false;
                   return;
                end
        end 
       
        Ay = y2 - y1;
        By = y3 - y4;

        if (Ay < 0)
                y1lo = y2;
                y1hi = y1;
        else
                y1hi = y2;
                y1lo = y1;
	end
	
        if (By > 0)
                if (y1hi < y4 || y3 < y1lo)
                   collision = false;
                   return;
                end
        elseif (y1hi < y3 || y4 < y1lo)
                collision = false;
                return;
	end        

        f = Ay * Bx - Ax * By; 
        if (f == 0)
        	collision = false;
        	return;
        end
        
        Cx = x1 - x3;
        Cy = y1 - y3;

        d = By * Cx - Bx * Cy; %alpha numerator
        % alpha tests
        if (f > 0)
                if (d < 0 || d > f)
                   collision = false;
                   return;
                end
        elseif (d > 0 || d < f)
            collision = false;
            return;
        end
        
        e = Ax * Cy - Ay * Cx; % beta numerator
        % beta tests
        if (f > 0)
                if (e < 0 || e > f)
                    collision = false;
                    return;
                end
        elseif (e > 0 || e < f)
             collision = false;
             return;
        end

	collision = true;
end
