function  [S,E,count,count_] = swapp(S,E,count,count_)
 i = 1;    
 j = 1;
 k = 1;
 d = count - count_;
 
 while j <= count_
     
        M.x(j) = E.x(j);
        M.y(j) = E.y(j);
        M.prex(j) = E.prex(j);
        M.prey(j) = E.prey(j);
        M.totalcost(j) = E.totalcost(j);
        M.pre(j) = E.pre(j);   
        
        j = j + 1;
        
 end 
 
 
 
 while i <= count  
  
        E.x(i) = S.x(i);
        E.y(i) = S.y(i);
        E.prex(i) = S.prex(i);
        E.prey(i) = S.prey(i);
        E.totalcost(i) = S.totalcost(i);
        E.pre(i) = S.pre(i);
   
        i = i + 1;
 end

while k <= count_ 
        S.x(k) = M.x(k);
        S.y(k) = M.y(k);
        S.prex(k) = M.prex(k);
        S.prey(k) = M.prey(k);
        S.totalcost(k) = M.totalcost(k);
        S.pre(k) = M.pre(k);
        
        k = k + 1;
end
 
for i = 1 : d
    S.x(count_ + 1) = [];
    S.y(count_ + 1) = [];
    S.prex(count_ + 1) = [];
    S.prey(count_ + 1) = [];
    S.totalcost(count_ + 1) = [];
    S.pre(count_ + 1) = []; 
end
 
 d = count_;
 count_ = count;
 count = d;
 
 
end

