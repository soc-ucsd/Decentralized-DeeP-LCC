function [IndSym,IndDiag,IndOffDiag,ShrinkIndDiag,ShrinkIndOffDiag,IndOffDiagCounter] = SymmetricIndices(n,ShrinkOrNot)
    %This generates some indices a symmetric matrix
    %Authors: Feng-Yi Liao & Yang Zheng
    %         SOC Lab @UC San Diego
    %[Input]  n: dimension of a matrix
    %         ShrinkOrNot: True or false. (If true, the function will also compute the indices after shrinking. Please set it to be "false" for efficiency)
    %
    %[Output]
    %       IndSym: Indices of the lower triangle parts (including diagonal elements)
    %       IndDiag: Indices of the diagonal elements
    %       IndOffDiag: Indices of the lower triangle parts (not including diagonal elements)
    %       IndOffDiagCounter: Indices of the upper triangle parts (the corresponding symmetric parts of IndOffDiag)
    %
    %[Example]
    %       suppose n = 3; 
    %       The indices of a 3 x 3 matrix will be
    %       [1,4,7
    %        2,5,8
    %        3,6,9]
    %[Output]:
    %       IndSym            = [1,2,3,5,6,9]
    %       IndDiag           = [1,5,9]
    %       IndOffDiag        = [2,3,6]
    %       IndOffDiagCounter = [4,7,8]


    %A is just a matrix variable
    A = ones(n);
    A = tril(A);
    IndSym = find(A);
    
    A = tril(A,-1);
    IndOffDiag = find(A);
    [row,col] = ind2sub([n,n],IndOffDiag);
    IndOffDiagCounter = sub2ind([n,n],col,row);
    
    
    A = diag(ones(1,n));
    IndDiag = find(A);
    
    ShrinkIndOffDiag = [];
    ShrinkIndDiag = [];
    
    k= 1;
    if ShrinkOrNot
        for i = 1:n
           for j=i:n
               if (j==i)
                   ShrinkIndDiag = [ShrinkIndDiag,k];
               else
                   ShrinkIndOffDiag = [ShrinkIndOffDiag,k];
               end
               k = k+1;
           end
        end
    end    

end