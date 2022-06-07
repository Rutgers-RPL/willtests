#ifndef MEFKcRP_H
#define MEFKcRP_H

#include <Arduino.h>

  // PRIVATE VARIABLES
  // quaternion describing the orientation (q1,q2,q3,q4)=(qx,qy,qz,qw)
  // (rotation that transform vectors from the sensor reference frame, to the external reference frame)
float q[4];
  // angular velocity (rad/s)
float w[3];
  // covariance matrix
float P[36];
  // covariance matrix of the angular velocity noise (rad^2/s^3)
float Qw[9];
  // covariance matrix of the acceleration noise (g^2)
float Qa[9];
  // covariance matrix of the angular velocity measurement noise (rad^2/s^2)
float Rw[9];
  // covariance matrix of the acceleration measurement noise (g^2)
float Ra[9];
  // use or not the chart update
bool chartUpdate = true;

  
  // PROTECTED ABSTRACT METHODS
  
  // Method: fC2M
  // defines the map from the chart points, to the manifold points (through the delta quaternion)
  // inputs:
  //  e: point of the Euclidean space that we want to map to a unit quaternion
  // outputs:
  //  delta: quaternion mapped with the e point
  void fC2M(float e[], float delta[]) {
    float aux = 1.0/sqrt( 4.0 + e[0]*e[0] + e[1]*e[1] + e[2]*e[2] );
    delta[0] = 2.0*aux;
    delta[1] = e[0]*aux;
    delta[2] = e[1]*aux;
    delta[3] = e[2]*aux;
  }
  
  // Method: chartUpdateMatrix
  // this function defines the transformation on the covariance matrix
  // when it is redefined from the chart centered in q quaternion, to the
  // chart centered in p quaternion, being them related by  p = q * delta
  // inputs:
  //  delta: quaternion used to update the quaternion estimation
  // outputs:
  //  G: transformation matrix to update the covariance matrix
  void chartUpdateMatrix(float delta[], float G[]) {
    // we will not use delta again in this update, so we transform it to save computations
    delta[1] *= delta[0];
    delta[2] *= delta[0];
    delta[3] *= delta[0];
    delta[0] *= delta[0];
    // now we build the transformation matrix
    G[0] = delta[0];     G[3] = delta[3];     G[6] = -delta[2];
    G[1] = -delta[3];    G[4] = delta[0];     G[7] = delta[1];
    G[2] = delta[2];     G[5] = -delta[1];    G[8] = delta[0];
  }
  
  
  // PRIVATE STATIC METHODS
  
  // Method: Cholesky
  // performs the Cholesky decomposition of a positive definite matrix ( S = L*L' )
  // inputs:
  //  S: 6x6 positive definite matrix to be decomposed (must be stored by columns)
  // outputs:
  //  S: the lower triangular matrix L (6x6) is overwritten in S (is stored by columns)
  static void Cholesky(float S[]){
    // for each column
    for(int j=0; j<6; j++){
      float sum = 0.0;  //sum for the diagonal term
      // we first fill with 0.0 until diagonal
      for(int i=0; i<j; i++){
        S[i+j*6] = 0.0;
        //we can compute this sum at the same time
        sum += S[j+i*6]*S[j+i*6];
      }
      // now we compute the diagonal term
      S[j*7] = sqrt( S[j*7] - sum ); //S[j+j*m] = sqrt( S[j+j*m] - sum );
      // finally we compute the terms below the diagonal
      for(int i=j+1; i<6; i++){
        //first the sum
        sum = 0.0;
        for(char k=0; k<j; k++){
          sum += S[i+k*6]*S[j+k*6];
        }
        //after the non-diagonal term
        S[i+j*6] = ( S[i+j*6] - sum )/S[j*7];
      }
    }//end j
  }
  
  // Method: solve
  // solves the system of linear equations  K*S = M  for K
  // inputs:
  //  S: 6x6 positive definite matrix stored by columns
  //  M: 6x6 matrix stored by rows
  // outputs:
  //  M: K (6x6) is stored by rows in the M memory space
  static void solve(float S[], float M[]){
    // we first compute the Cholesky decomposition for transform the system from  K*S = M  into K*L*L' = M
    Cholesky(S);
    
    float y[6];
    // then we take each pair of rows of K and M independently
    for(int i=0; i<6; i++){
      // first we solve (y*L' = M)
      for(int j=0; j<6; j++){
        float sum = 0.0;
        for(int k=0; k<j; k++){
          sum += y[k]*S[j+k*6];
        }
        y[j] = ( M[i*6+j] - sum )/S[j*7];
      }
      // now we solve (Ki*L = y)
      for(int j=5; j>-1; j--){
        float sum = 0.0;
        for(int k=j+1; k<6; k++){
          sum += M[i*6+k]*S[k+j*6];
        }
        M[i*6+j] = ( y[j] - sum )/S[j*7];
      }
    }
  }

void OrientationEstimator(){
    q[0] = 1.0;   q[1] = 0.0;   q[2] = 0.0;   q[3] = 0.0;
    // it is necessary to set an angular velocity different from 0.0 to break the symmetry
    // otherwise, the MUKF could not converge (especially when we apply the "reset operation" with the RV chart)
    w[0] = 1.0e-10;   w[1] = 1.0e-10;   w[2] = 1.0e-10;
    
    for(int k=0; k<36; k++) P[k] = 0.0;
    for(int k=0; k<36; k+=7) P[k] = 1.0e2;
    P[2+2*6] = 1.0e-16;
    
    for(int k=0; k<9; k++){
      Qw[k] = 0.0;
      Qa[k] = 0.0;
      Rw[k] = 0.0;
      Ra[k] = 0.0;
    }
    for(int k=0; k<9; k+=4){
      Qw[k] = 1.0e1;
      Qa[k] = 1.0e-2;
      Rw[k] = 1.0e-3;
      Ra[k] = 1.0e-3;
    }
  }
  
  void get_q(float qOut[]) {
    for(int i=0; i<4; i++) qOut[i] = q[i];
  }
  
  void set_q(float qIn[]) {
    for(int i=0; i<4; i++) q[i] = qIn[i];
    for(int k=0; k<36; k++) P[k] = 0.0;
    for(int k=0; k<36; k+=7) P[k] = 1.0e-8;
    P[2+2*6] = 1.0e-16;
  }
  
  void reset_orientation() {
    q[0] = 1.0;   q[1] = 0.0;   q[2] = 0.0;   q[3] = 0.0;
    w[0] = 0.0;   w[1] = 0.0;   w[2] = 0.0;
    
    for(int k=0; k<36; k++) P[k] = 0.0;
    for(int k=0; k<36; k+=7) P[k] = 1.0e2;
    P[2+2*6] = 1.0e-16;
  }
  
  void set_Qw(float QwIn){
    for(int k=0; k<9; k++) Qw[k] = 0.0;
    for(int k=0; k<9; k+=4) Qw[k] = QwIn;
  }
  
  void set_Qa(float QaIn) {
    for(int k=0; k<9; k++) Qa[k] = 0.0;
    for(int k=0; k<9; k+=4) Qa[k] = QaIn;
  }
  
  void set_Rw(float RwIn) {
    for(int k=0; k<9; k++) Rw[k] = 0.0;
    for(int k=0; k<9; k+=4) Rw[k] = RwIn;
  }
  
  void set_Ra(float RaIn) {
    for(int k=0; k<9; k++) Ra[k] = 0.0;
    for(int k=0; k<9; k+=4) Ra[k] = RaIn;
  }
  
  void set_chartUpdate(bool chartUpdateIn) {
    chartUpdate = chartUpdateIn;
  }
  
  // ABSTRACT METHODS
  void updateIMU(float am[3], float wm[3], float dt){
    // we compute the state prediction
    float wnorm = sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    float qw[4];
    if( wnorm != 0.0 ){
      float wdt05 = 0.5*wnorm*dt;
      float swdt = sin(wdt05)/wnorm;
      // THIS LINE CRASHES PROGRAM IF TOO FAST
      qw[0] = cos(wdt05);

      qw[1] = w[0]*swdt;
      qw[2] = w[1]*swdt;
      qw[3] = w[2]*swdt;
    }else{
      qw[0] = 1.0;
      qw[1] = 0.0;
      qw[2] = 0.0;
      qw[3] = 0.0;
    }
    
    float qp[4];
    qp[0] = q[0]*qw[0]  -  q[1]*qw[1]  -  q[2]*qw[2] - q[3]*qw[3];
    qp[1] = q[0]*qw[1]  +  qw[0]*q[1]  +  q[2]*qw[3] - q[3]*qw[2];
    qp[2] = q[0]*qw[2]  +  qw[0]*q[2]  +  q[3]*qw[1] - q[1]*qw[3];
    qp[3] = q[0]*qw[3]  +  qw[0]*q[3]  +  q[1]*qw[2] - q[2]*qw[1];
    
    // we compute the covariance matrix for the state prediction
    for(int j=0; j<3; j++) {
      for(int i=0; i<3; i++) P[i+j*6] += Qw[i+j*3]*dt*dt*dt/3;
    }
    for(int j=0; j<3; j++) {
      for(int i=3; i<6; i++) P[i+j*6] -= Qw[i-3+(j)*3]*dt*dt/2;
    }
    for(int j=3; j<6; j++) {
      for(int i=0; i<3; i++) P[i+j*6] -= Qw[i+(j-3)*3]*dt*dt/2;
    }
    for(int j=3; j<6; j++) {
      for(int i=3; i<6; i++) P[i+j*6] += Qw[i-3+(j-3)*3]*dt;
    }
    
    float F[9];
    F[0] = -qw[2]*qw[2]-qw[3]*qw[3];    F[3] = qw[1]*qw[2]+qw[3]*qw[0];     F[6] = qw[1]*qw[3]-qw[2]*qw[0];
    F[1] = qw[1]*qw[2]-qw[3]*qw[0];     F[4] = -qw[1]*qw[1]-qw[3]*qw[3];    F[7] = qw[2]*qw[3]+qw[1]*qw[0];
    F[2] = qw[1]*qw[3]+qw[2]*qw[0];     F[5] = qw[2]*qw[3]-qw[1]*qw[0];     F[8] = -qw[1]*qw[1]-qw[2]*qw[2];
    
    F[0] += F[0] + 1.0;    F[3] += F[3];          F[6] += F[6];
    F[1] += F[1];          F[4] += F[4] + 1.0;    F[7] += F[7];
    F[2] += F[2];          F[5] += F[5];          F[8] += F[8] + 1.0;
    
    float M[36];
    M[0] = F[0];    M[6] = F[3];    M[12] = F[6];    M[18] = dt;     M[24] =0.0;     M[30] = 0.0;
    M[1] = F[1];    M[7] = F[4];    M[13] = F[7];    M[19] = 0.0;    M[25] = dt;     M[31] = 0.0;
    M[2] = F[2];    M[8] = F[5];    M[14] = F[8];    M[20] = 0.0;    M[26] = 0.0;    M[32] = dt;
    M[3] = 0.0;     M[9] = 0.0;     M[15] = 0.0;     M[21] = 1.0;    M[27] = 0.0;    M[33] = 0.0;
    M[4] = 0.0;     M[10] = 0.0;    M[16] = 0.0;     M[22] = 0.0;    M[28] = 1.0;    M[34] = 0.0;
    M[5] = 0.0;     M[11] = 0.0;    M[17] = 0.0;     M[23] = 0.0;    M[29] = 0.0;    M[35] = 1.0;
    
    float S[36];
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++){
        float sum = 0.0;
        for(int k=0; k<6; k++) sum += P[i+k*6]*M[j+k*6];
        S[i*6+j] = sum;
      }
    }
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++){
        float sum = 0.0;
        for(int k=0; k<6; k++) sum += M[i+k*6]*S[k*6+j];
        P[i+j*6] = sum;
      }
    }
    
    // we compute the measurement prediction
    float ap[] = { qp[1]*qp[3]-qp[2]*qp[0] , qp[2]*qp[3]+qp[1]*qp[0] , -qp[1]*qp[1]-qp[2]*qp[2] };
    ap[0] += ap[0];
    ap[1] += ap[1];
    ap[2] += ap[2] + 1.0;
    
    F[0] = 0.0;       F[3] = -ap[2];    F[6] = ap[1];
    F[1] = ap[2];     F[4] = 0.0;       F[7] = -ap[0];
    F[2] = -ap[1];    F[5] = ap[0];     F[8] = 0.0;
    
    float H[36];
    H[0] = F[0];    H[6] = F[3];    H[12] = F[6];    H[18] = 0.0;    H[24] = 0.0;    H[30] = 0.0;
    H[1] = F[1];    H[7] = F[4];    H[13] = F[7];    H[19] = 0.0;    H[25] = 0.0;    H[31] = 0.0;
    H[2] = F[2];    H[8] = F[5];    H[14] = F[8];    H[20] = 0.0;    H[26] = 0.0;    H[32] = 0.0;
    H[3] = 0.0;     H[9] = 0.0;     H[15] = 0.0;     H[21] = 1.0;    H[27] = 0.0;    H[33] = 0.0;
    H[4] = 0.0;     H[10] = 0.0;    H[16] = 0.0;     H[22] = 0.0;    H[28] = 1.0;    H[34] = 0.0;
    H[5] = 0.0;     H[11] = 0.0;    H[17] = 0.0;     H[23] = 0.0;    H[29] = 0.0;    H[35] = 1.0;
    
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++){
        float sum = 0.0;
        for(int k=0; k<6; k++) sum += P[i+k*6]*H[j+k*6];
        M[i*6+j] = sum;
      }
    }
    
    for(int j=0; j<6; j++){
      for(int i=0; i<6; i++){
        float sum = 0.0;
        for(int k=0; k<6; k++) sum += H[i+k*6]*M[k*6+j];
        S[i+j*6] = sum;
      }
    }
    
    for(int j=0; j<3; j++){
      for(int i=0; i<3; i++) S[i+j*6] += Qa[i+j*3] + Ra[i+j*3];
    }
    for(int j=3; j<6; j++){
      for(int i=3; i<6; i++) S[i+j*6] += Rw[i-3+(j-3)*3];
    }
    
    // now we can compute the gain
    solve(S, M);  // now K is stored in M
    
    // and update the state in the chart
    float dy[] = { am[0]-ap[0] , am[1]-ap[1] , am[2]-ap[2] , wm[0]-w[0] , wm[1]-w[1] , wm[2]-w[2] };
    
    float dx[6];
    for(int i=0; i<6; i++){
      float sum = 0.0;
      for(int j=0; j<6; j++) sum += M[i*6+j]*dy[j];
      dx[i] = sum;
    }
    
    // the updated point in the chart is mapped to a quaternion
    fC2M(dx, qw);  // now delta is stored in qw
    
    q[0] = qp[0]*qw[0] - qp[1]*qw[1] - qp[2]*qw[2] - qp[3]*qw[3];
    q[1] = qp[0]*qw[1]  +  qw[0]*qp[1]  +  qp[2]*qw[3] - qp[3]*qw[2];
    q[2] = qp[0]*qw[2]  +  qw[0]*qp[2]  +  qp[3]*qw[1] - qp[1]*qw[3];
    q[3] = qp[0]*qw[3]  +  qw[0]*qp[3]  +  qp[1]*qw[2] - qp[2]*qw[1];
    
    // and the angular velocity is updated in the usual way
    w[0] += dx[3];
    w[1] += dx[4];
    w[2] += dx[5];
    
    // the covariance matrix is updated in the chart centered in qp
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++){
        float sum = 0.0;
        for(int k=0; k<6; k++) sum -= M[i*6+k]*H[k+j*6];
        S[i*6+j] = sum;
      }
    }
    for(int k=0; k<36; k+=7) S[k] += 1.0;
    
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++){
        float sum = 0.0;
        for(int k=0; k<6; k++) sum += S[i*6+k]*P[k+j*6];
        M[i+j*6] = sum;
      }
    }
    
    if( chartUpdate ){
      // finally we update the covariance matrix from the chart centered in qp
      // quaternion to the chart centered in the updated q quaternion
      chartUpdateMatrix( qw , H );  // now G is stored in H
      
      S[0] = H[0];    S[6] = H[3];    S[12] = H[6];    S[18] = 0.0;    S[24] = 0.0;    S[30] = 0.0;
      S[1] = H[1];    S[7] = H[4];    S[13] = H[7];    S[19] = 0.0;    S[25] = 0.0;    S[31] = 0.0;
      S[2] = H[2];    S[8] = H[5];    S[14] = H[8];    S[20] = 0.0;    S[26] = 0.0;    S[32] = 0.0;
      S[3] = 0.0;     S[9] = 0.0;     S[15] = 0.0;     S[21] = 1.0;    S[27] = 0.0;    S[33] = 0.0;
      S[4] = 0.0;     S[10] = 0.0;    S[16] = 0.0;     S[22] = 0.0;    S[28] = 1.0;    S[34] = 0.0;
      S[5] = 0.0;     S[11] = 0.0;    S[17] = 0.0;     S[23] = 0.0;    S[29] = 0.0;    S[35] = 1.0;
      
      for(int i=0; i<6; i++){
        for(int j=0; j<6; j++){
          float sum = 0.0;
          for(int k=0; k<6; k++) sum += M[i+k*6]*S[j+k*6];
          H[i*6+j] = sum;
        }
      }
      
      for(int j=0; j<6; j++){
        for(int i=0; i<6; i++){
          float sum = 0.0;
          for(int k=0; k<6; k++) sum += S[i+k*6]*H[k*6+j];
          M[i+j*6] = sum;
        }
      }
    } //chartUpdate
    
    // we avoid numerical instabilities
    float qnorm = sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
    q[0] /= qnorm;
    q[1] /= qnorm;
    q[2] /= qnorm;
    q[3] /= qnorm;
    
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++) P[i+j*6] = 0.5*( M[i+j*6] + M[j+i*6] );
    }
  }

#endif