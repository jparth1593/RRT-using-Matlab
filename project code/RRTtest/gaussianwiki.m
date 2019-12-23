X = -5:0.1:5;
Y = -5:0.1:5;

%2-d Mean and covariance matrix
MeanVec = [0 0];
CovMatrix = [1 0.6; 0.6 2];

%Get the 1-d PDFs for the "walls"
Z_x = normpdf(X,MeanVec(1), sqrt(CovMatrix(1,1)));
Z_y = normpdf(Y,MeanVec(2), sqrt(CovMatrix(2,2)));

%Get the 2-d samples for the "floor"
Samples = mvnrnd(MeanVec, CovMatrix, 5000);

%Get the sigma ellipses by transform a circle by the cholesky decomp
L = chol(CovMatrix,'lower');
t = linspace(0,2*pi,100); %Our ellipse will have 100 points on it
C = [cos(t) ; sin(t)]; %A unit circle
E1 = 1*L*C; E2 = 2*L*C; E3 = 3*L*C; %Get the 1,2, and 3-sigma ellipses

figure; hold on; 
%Plot the samples on the "floor"
plot3(Samples(:,1),Samples(:,2),zeros(size(Samples,1),1),'k.','MarkerSize',3)