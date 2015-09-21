#include "Def4PlanTrajectory.h"


void TRAJSENTDOWN::reset()
{
    pathParameter.reset();
    velocityParameter.reset();

    S.clear();
    X.clear();
    Y.clear();
    H.clear();
    K.clear();
    DK.clear();
    V.clear();

    numOfPoints = 0;

    isValid = false;
}


void trajplan_trajectory_plus::reset()
{
    X.clear();
    Y.clear();
    H.clear();
    K.clear();
    DK.clear();
    V.clear();

    numOfPoints = 0;
}


double trajplan_trajectory_plus::getProjectedSignedDistanceByCoordinates(double x, double y)
{
    float minDist1 = GSL_POSINF;
    int s_Id1 = -1;
    float minDist2 = GSL_POSINF;
    int s_Id2 = -1;

    for(unsigned int j = 0; j < X.size(); j++ )
    {
        double dist = hypot(x - X[j], y - Y[j]);
        if(minDist1 > dist)
        {
            minDist2 = minDist1;
            minDist1 = dist;
            s_Id2 = s_Id1;
            s_Id1 = j;
        }
        else if(minDist2 > dist)
        {
            minDist2 = dist;
            s_Id2 = j;
        }
    }

    double angle1 = atan2(y - Y[s_Id1], x - X[s_Id1]);
    double angle2;

    if(s_Id1 >= numOfPoints - 1)
    {
        //If the nearest is the last point in ci_Route ...
        angle2 = atan2(Y[s_Id1] - Y[s_Id1-1],
                       X[s_Id1] - X[s_Id1-1]);
    }
    else
    {
        angle2 = atan2(Y[s_Id1+1] - Y[s_Id1],
                       X[s_Id1+1] - X[s_Id1]);
    }

    // Calculate the real dist
    double a = hypot(Y[s_Id1] - y,
                     X[s_Id1] - x);
    double b = hypot(Y[s_Id2] - y,
                     X[s_Id2] - x);
    double c = hypot(Y[s_Id2] - Y[s_Id1],
                     X[s_Id2] - X[s_Id1]);
    double p = (a+b+c)/2;
    double area = sqrt(p*(p-a)*(p-b)*(p-c));

    double minDist = 2*area/c;

    double dist;
    if (angle2 >= 0)
    {
        if (angle1 >= 0)
        { // if both in "top half" of atan2 plot
            if((angle1 - angle2) >= 0)
            {
                //left side
                dist = minDist;
            }
            else
            {
                // right side
                dist = -minDist;
            }
        }
        if (angle1 < 0)
        { // angle2 in top half, angle1 in bottom half
            if(angle1 >= angle2- M_PI)
            {
                // right side
                dist = -minDist;
            }
            else
            {
                //left side
                dist = minDist;
            }
        }
    }
    else
    {
        if (angle1 >= 0)
        { // angle2 in bottom half, angle1 in top half
            if (angle1 <= (angle2 + M_PI))
            {
                //left side
                dist = minDist;
            }
            else
            {
                // right side
                dist = -minDist;
            }
        }
        if (angle1 < 0)
        { // angle1 and angle2 both in bottom half
            if (angle1 >= angle2)
            {
                //left side
                dist = minDist;
            }
            else
            {
                // right side
                dist = -minDist;
            }
        }
    }

    return dist;
}


State trajplan_trajectory_plus::getProjectedTrajectoryStateByCoordinates(double x, double y)
{
    double minDist = GSL_POSINF;
    int minId = -1;

    for(int ii=0; ii<numOfPoints; ii++) {
        double dist = hypot(x - X[ii], y - Y[ii]);
        if(dist < minDist) {
            minDist = dist;
            minId = ii;
        }
    }

    State tmp;
    tmp.spatial.x = X[minId];
    tmp.spatial.y = Y[minId];
    tmp.spatial.theta = H[minId];
    tmp.spatial.k = K[minId];
    tmp.spatial.dk = DK[minId];

    tmp.temporal.v = V[minId];

    if( minId+1 < numOfPoints )
        tmp.temporal.a = (V[minId+1] - V[minId]) / TRAJPLAN_TRAJECTORY_DT;
    else
        tmp.temporal.a = 0.0;

    return tmp;
}

/************************************ DEFINITIONs for struct PATH **************************************/
void Path::clear()
{
    s.clear();
    x.clear();
    y.clear();
    theta.clear();
    k.clear();
    dk.clear();
    ddk.clear();

    numOfPoints = 0;

    isValid = false;
}

void Path::resize(int pointNum)
{
    s.resize(pointNum);
    x.resize(pointNum);
    y.resize(pointNum);
    theta.resize(pointNum);
    k.resize(pointNum);
    dk.resize(pointNum);
    ddk.resize(pointNum);

    numOfPoints = pointNum;

    isValid = false;
}

int Path::getProjIdxByS(double s)
{
    int minId = -1;
    double minSDist = GSL_POSINF;

    for(int ii=0; ii<numOfPoints; ii++)
    {
        double SDist = fabs(s - this->s[ii]);
        if(SDist < minSDist)
        {
            minSDist = SDist;
            minId = ii;
        }
    }

    return minId;
}

/************************************ DEFINITIONs for struct VELOCITY **************************************/
void Velocity::clear()
{
    t.clear();
    s.clear();
    v.clear();
    a.clear();
    numOfPoints = 0;

    isValid = false;
}

void Velocity::resize(int pointNum)
{
    t.resize(pointNum);
    s.resize(pointNum);
    v.resize(pointNum);
    a.resize(pointNum);
    numOfPoints = pointNum;

    isValid = false;
}


/************************************ DEFINITIONs for struct PATH_PARAMETER_XY5 **************************************/
void PATH_PARAMETER_XY5::reset()
{
    s0.reset();
    sf.reset();
    p[0] = 0.0; p[1] = 0.0; p[2] = 0.0; p[3] = 0.0; p[4] = 0.0; p[5] = 0.0;
    length = 0.0;
    isValid = false;

    xf = 0.0;
    helpingPath.clear();
}

std::pair<double,double> PATH_PARAMETER_XY5::getXYByS(double s)
{
    int idOnHelpingPath = getIdOnHelpingPathByS(s);

    std::pair<double,double> tmpXY;
    tmpXY.first = helpingPath.x[idOnHelpingPath];
    tmpXY.second = helpingPath.y[idOnHelpingPath];

    return tmpXY;
}

double PATH_PARAMETER_XY5::getBiasThetaByS(double s)
{
    int idOnHelpingPath = getIdOnHelpingPathByS(s);

    return helpingPath.theta[idOnHelpingPath] - s0.theta;
}

double PATH_PARAMETER_XY5::getCurvatureByS(double s)
{
    int idOnHelpingPath = getIdOnHelpingPathByS(s);

    return helpingPath.k[idOnHelpingPath];
}

double PATH_PARAMETER_XY5::getDCurvatureByS(double s)
{
    int idOnHelpingPath = getIdOnHelpingPathByS(s);

    return helpingPath.dk[idOnHelpingPath];
}

double PATH_PARAMETER_XY5::getDDCurvatureByS(double s)
{
    int idOnHelpingPath = getIdOnHelpingPathByS(s);

    return helpingPath.ddk[idOnHelpingPath];
}

void PATH_PARAMETER_XY5::findPathParameter(StateSpatial startState, StateSpatial endState)
{
    s0 = startState;
    sf = endState;

    double Kappa_0 = s0.k;

    double x_f = cos(s0.theta)*(sf.x-s0.x) + sin(s0.theta)*(sf.y-s0.y);
    double y_f = cos(s0.theta)*(sf.y-s0.y) - sin(s0.theta)*(sf.x-s0.x);
    double theta_f = sf.theta - s0.theta;
    double Kappa_f = sf.k;

    //    qDebug("transformed state = (%f, %f, %f, %f)", x_f, y_f, theta_f, Kappa_f);

    //theta
    double theta1,theta2;
    theta1 = fmod(theta_f+8*M_PI,2*M_PI);
    theta2 = theta1-2*M_PI;
    if(fabs(theta1)<=fabs(theta2)) { theta_f=theta1; }
    else { theta_f=theta2; }

    p[0] = 0;
    p[1] = 0;
    p[2] = Kappa_0/2;

    if( fabs(x_f) >= 1e-3 && fabs(y_f) >= 1e-3 )
    {
        double a_3n = (20*y_f - 8*theta_f*x_f - 3*Kappa_0*x_f*x_f + Kappa_f*x_f*x_f)/(2*y_f);
        double a_4n = -(30*y_f - 14*theta_f*x_f - 3*Kappa_0*x_f*x_f + 2*Kappa_f*x_f*x_f)/(2*y_f);
        double a_5n = (12*y_f - 6*theta_f*x_f - Kappa_0*x_f*x_f + Kappa_f*x_f*x_f)/(2*y_f);

        p[3] = a_3n * y_f / (x_f*x_f*x_f);
        p[4] = a_4n * y_f / (x_f*x_f*x_f*x_f);
        p[5] = a_5n * y_f / (x_f*x_f*x_f*x_f*x_f);
    }
    else
    {
        p[3] = 0.0;
        p[4] = 0.0;
        p[5] = 0.0;
    }

    isValid = true;


    // Only for this particular path primitive
    xf = x_f;

    double dx = xf / (POINTNUM_INTERNAL-1);

    vector<double> x_local; x_local.resize( POINTNUM_INTERNAL );
    vector<double> y_local; y_local.resize( POINTNUM_INTERNAL );
    vector<double> theta_local; theta_local.resize( POINTNUM_INTERNAL );
    vector<double> k_local; k_local.resize( POINTNUM_INTERNAL );

    for(int ii = 0; ii < POINTNUM_INTERNAL; ii++)
    {
        double x = dx * ii;
        double y = p[5] * pow(x, 5) + p[4] * pow(x, 4) + p[3] * pow(x, 3) + p[2] * pow(x, 2) + p[1] * x + p[0];
        double dy = 5 * p[5] * pow(x, 4) + 4 * p[4] * pow(x, 3) + 3 * p[3] * pow(x, 2) + 2 * p[2] * x + p[1];
        double ddy = 20 * p[5] * pow(x, 3) + 12 * p[4] * pow(x, 2) + 6 * p[3] * x + 2 * p[2];

        x_local[ii] = x;
        y_local[ii] = y;
        theta_local[ii] = atan(dy);
        k_local[ii] = ddy / ( pow((1 + dy*dy), (3/2)) );
    }

    helpingPath.clear();
    helpingPath.resize(POINTNUM_INTERNAL);

    for(int ii = 0; ii < POINTNUM_INTERNAL; ii++)
    {
        helpingPath.x[ii] = s0.x + x_local[ii] * cos(s0.theta) - y_local[ii] * sin(s0.theta);
        helpingPath.y[ii] = s0.y + x_local[ii] * sin(s0.theta) + y_local[ii] * cos(s0.theta);
        helpingPath.theta[ii] = s0.theta + theta_local[ii];
        helpingPath.k[ii] = k_local[ii];
        helpingPath.dk[ii] = 0;

        if(ii==0) {helpingPath.s[ii] = 0;}
        else
        {
            helpingPath.s[ii] = helpingPath.s[ii-1] +
                    hypot(helpingPath.x[ii]-helpingPath.x[ii-1], helpingPath.y[ii]-helpingPath.y[ii-1]);
        }
    }

    length = helpingPath.s.back();
}

Path PATH_PARAMETER_XY5::plotPath(int pointNum)
{
    Path tmpPath;

    if( ! this->isValid )
    {
        tmpPath.isValid = false;
        return tmpPath;
    }

    tmpPath.resize(pointNum);

    double dx = xf / (pointNum-1);

    vector<double> x_local; x_local.resize( pointNum );
    vector<double> y_local; y_local.resize( pointNum );
    vector<double> theta_local; theta_local.resize( pointNum );
    vector<double> k_local; k_local.resize( pointNum );

    for(int ii = 0; ii < pointNum; ii++)
    {
        double x = dx * ii;
        double y = p[5] * pow(x, 5) + p[4] * pow(x, 4) + p[3] * pow(x, 3) + p[2] * pow(x, 2) + p[1] * x + p[0];
        double dy = 5 * p[5] * pow(x, 4) + 4 * p[4] * pow(x, 3) + 3 * p[3] * pow(x, 2) + 2 * p[2] * x + p[1];
        double ddy = 20 * p[5] * pow(x, 3) + 12 * p[4] * pow(x, 2) + 6 * p[3] * x + 2 * p[2];

        x_local[ii] = x;
        y_local[ii] = y;
        theta_local[ii] = atan(dy);
        k_local[ii] = ddy / ( pow((1 + dy*dy), (3/2)) );
    }

    for(int ii = 0; ii < pointNum; ii++)
    {
        tmpPath.x[ii] = s0.x + x_local[ii] * cos(s0.theta) - y_local[ii] * sin(s0.theta);
        tmpPath.y[ii] = s0.y + x_local[ii] * sin(s0.theta) + y_local[ii] * cos(s0.theta);
        tmpPath.theta[ii] = s0.theta + theta_local[ii];
        tmpPath.k[ii] = k_local[ii];
        tmpPath.dk[ii] = 0;

        if(ii==0) {tmpPath.s[ii] = 0;}
        else
        {
            tmpPath.s[ii] = tmpPath.s[ii-1] +
                    hypot(tmpPath.x[ii]-tmpPath.x[ii-1], tmpPath.y[ii]-tmpPath.y[ii-1]);
        }
    }

    tmpPath.isValid = true;

    return tmpPath;
}

int PATH_PARAMETER_XY5::getIdOnHelpingPathByS(double s)
{
    int minId = -1;
    double minDistS = GSL_POSINF;
    for(int ii=0; ii<helpingPath.numOfPoints; ii++)
    {
        double distS = fabs(helpingPath.s[ii] - s);
        if(distS < minDistS)
        {
            minDistS = distS;
            minId = ii;
        }
    }

    return minId;
}

/************************************ DEFINITIONs for struct PathParameterArc **************************************/

void PathParameterArc::reset()
{
    s0.reset();
    sf.reset();

    p[0] = 0.0; p[1] = 0.0; p[2] = 0.0; p[3] = 0.0; p[4] = 0.0; p[5] = 0.0;
    length = 0.0;

    order = 3;
    isValid = false;
}

void PathParameterArc::findPathParameter(StateSpatial startState, StateSpatial endState, int order)
{
    this->s0 = startState;
    this->sf = endState;
    this->order = order;

    StateSpatial transformEndState;
    transformEndState.x = cos(s0.theta)*(sf.x-s0.x) + sin(s0.theta)*(sf.y-s0.y);
    transformEndState.y = cos(s0.theta)*(sf.y-s0.y) - sin(s0.theta)*(sf.x-s0.x);
    transformEndState.theta = sf.theta-s0.theta;
    transformEndState.k = sf.k;

    //theta
    double tmpTheta = transformEndState.theta;
    while(tmpTheta < 0.0)
        tmpTheta += 2*M_PI;
    transformEndState.theta=fmod(tmpTheta,2*M_PI);
    if( transformEndState.theta >= M_PI )
        transformEndState.theta -= 2*M_PI;

    this->initParameter(s0, transformEndState);

    double distBtwnPoses_m = hypot( s0.x-sf.x, s0.y-sf.y );
    if( distBtwnPoses_m <= 0.1 ) {
        this->p[0] = 0.0;
        this->p[1] = 0.0;
        this->p[2] = 0.0;
        this->p[3] = 0.0;
        this->p[4] = 0.0;
        this->p[5] = 0.0;
        this->length = distBtwnPoses_m;
        isValid = true;
    }
    else {
        int i=0;
        StateSpatial tempX = calculateState();
        while( !this->isTermination(tempX, transformEndState) ) {
            if(i>=20) { break; }
            stepOptimize( tempX, transformEndState );
            tempX = calculateState();
            i++;
        }

        if(i>=20) {
            this->p[0] = 0.0;
            this->p[1] = 0.0;
            this->p[2] = 0.0;
            this->p[3] = 0.0;
            this->p[4] = 0.0;
            this->p[5] = 0.0;
            this->length = 0.0;
            isValid = false;
        }
        else {
            isValid = true;
        }
    }

    helpingPath.clear();
    helpingPath = plotPath(POINTNUM_INTERNAL);
}

void PathParameterArc::initParameter(StateSpatial startState, StateSpatial transformedState)
{
    double d = gsl_hypot(transformedState.x,transformedState.y);
    length = d;
    double k0 = startState.k;
    double dk0 = startState.dk;

    switch(this->order)
    {
    case 2:
        p[0]=k0;
        p[1]=0;
        p[2]=0;
        p[3]=0;
        p[4]=0;
        p[5]=0;
        break;
    case 3:
        p[0]=k0;
        p[1]=6*transformedState.theta/(this->length*this->length)-4*k0/this->length-2*transformedState.k/this->length;
        p[2]=3*(k0+transformedState.k)/(this->length*this->length)-6*transformedState.theta/(this->length*this->length*this->length);
        p[3]=0;
        p[4]=0;
        p[5]=0;
        break;
    case 4:
        p[0]=k0;
        p[1]=dk0;
        p[2]=-9*k0/gsl_pow_2(length)-3*transformedState.k/gsl_pow_2(length)-3*dk0/length+12*transformedState.theta/gsl_pow_3(length);
        p[3]=8*k0/gsl_pow_3(length)+4*transformedState.k/gsl_pow_3(length)+2*dk0/gsl_pow_2(length)-12*transformedState.theta/gsl_pow_4(length);
        p[4]=0;
        p[5]=0;
        break;
    }

    isValid = false;
}

StateSpatial PathParameterArc::calculateState()
{
    int N=POINTNUM_INTERNAL;
    int w;
    double s,f,g,theta;
    double x=0,y=0;
    for(int i=0;i<=N;i++)
    {
        if(i==0 || i==N) w=1;
        else if((i%2)==1) w=4;
        else w=2;
        s = length * i/(N-1);
        theta = getBiasThetaByS(s);

        f=cos(theta);
        g=sin(theta);
        x+=w*f;
        y+=w*g;
    }
    x*=length/(3*N);
    y*=length/(3*N);

    struct StateSpatial tempX;
    tempX.x = x;
    tempX.y = y;
    tempX.theta = getBiasThetaByS(this->length);
    tempX.k = getCurvatureByS(this->length);
    tempX.dk = getDCurvatureByS(this->length);

    //theta
    double theta1,theta2;
    theta1=fmod(tempX.theta+2*M_PI,2*M_PI);
    theta2=theta1-2*M_PI;
    if(fabs(theta1)<=fabs(theta2))
        tempX.theta=theta1;
    else
        tempX.theta=theta2;

    return tempX;
}

void PathParameterArc::stepOptimize(StateSpatial tempX, StateSpatial transformEndState)
{
    switch(this->order)
    {
    case 2:
    {
        double jacobi[9];
        int sig = 0;
        gsl_matrix *invertJac = gsl_matrix_alloc(3,3);
        gsl_vector *delta = gsl_vector_alloc(3);
        gsl_permutation *perm = gsl_permutation_alloc(3);

        calculateJacobi(jacobi);

        gsl_matrix_view jacobi_ = gsl_matrix_view_array(jacobi, 3, 3);
        double tempX__[]={tempX.x,tempX.y,tempX.theta};
        double endState__[]={transformEndState.x,transformEndState.y,transformEndState.theta};
        gsl_vector_view tempX_ = gsl_vector_view_array(tempX__, 3);
        gsl_vector_view endState_ = gsl_vector_view_array(endState__, 3);
        gsl_vector_sub(&tempX_.vector, &endState_.vector);

        gsl_linalg_LU_decomp(&jacobi_.matrix, perm, &sig);
        gsl_linalg_LU_invert(&jacobi_.matrix, perm, invertJac);
        gsl_blas_dgemv(CblasNoTrans, 1, invertJac, &tempX_.vector, 0, delta); //delta_p = inv(Jac)*(q-dest)';

        p[1] -= gsl_vector_get(delta,0);
        p[2] -= gsl_vector_get(delta,1);
        length -= gsl_vector_get(delta,2);

        gsl_permutation_free(perm);
        gsl_matrix_free(invertJac);
        gsl_vector_free(delta);
        break;
    }
    case 3:
    {
        double jacobi[16];
        int sig = 0;
        gsl_matrix *invertJac = gsl_matrix_alloc(4,4);
        gsl_vector *delta = gsl_vector_alloc(4);
        gsl_permutation *perm = gsl_permutation_alloc(4);

        this->calculateJacobi(jacobi);
        gsl_matrix_view jacobi_ = gsl_matrix_view_array(jacobi, 4, 4);
        double tempX__[]={tempX.x,tempX.y,tempX.theta,tempX.k};
        double endState__[]={transformEndState.x,transformEndState.y,transformEndState.theta,transformEndState.k};
        gsl_vector_view tempX_ = gsl_vector_view_array(tempX__,4);
        gsl_vector_view endState_ = gsl_vector_view_array(endState__,4);
        gsl_vector_sub(&tempX_.vector,&endState_.vector);

        gsl_linalg_LU_decomp(&jacobi_.matrix,perm,&sig);
        gsl_linalg_LU_invert(&jacobi_.matrix,perm,invertJac);
        gsl_blas_dgemv(CblasNoTrans,1,invertJac,&tempX_.vector,0,delta); //delta_p = inv(Jac)*(q-dest)';

        this->p[1]-=gsl_vector_get(delta,0);
        this->p[2]-=gsl_vector_get(delta,1);
        this->p[3]-=gsl_vector_get(delta,2);
        this->length-=gsl_vector_get(delta,3);

        gsl_permutation_free(perm);
        gsl_matrix_free(invertJac);
        gsl_vector_free(delta);
        break;
    }
    case 4:
    {
        double jacobi[16];
        int sig = 0;
        gsl_matrix *invertJac = gsl_matrix_alloc(4,4);
        gsl_vector *delta = gsl_vector_alloc(4);
        gsl_permutation *perm = gsl_permutation_alloc(4);

        calculateJacobi(jacobi);

        gsl_matrix_view jacobi_ = gsl_matrix_view_array(jacobi, 4, 4);
        double tempX__[]={tempX.x,tempX.y,tempX.theta,tempX.k};
        double endState__[]={transformEndState.x,transformEndState.y,transformEndState.theta,transformEndState.k};
        gsl_vector_view tempX_ = gsl_vector_view_array(tempX__,4);
        gsl_vector_view endState_ = gsl_vector_view_array(endState__,4);
        gsl_vector_sub(&tempX_.vector,&endState_.vector);

        gsl_linalg_LU_decomp(&jacobi_.matrix,perm,&sig);
        gsl_linalg_LU_invert(&jacobi_.matrix,perm,invertJac);
        gsl_blas_dgemv(CblasNoTrans,1,invertJac,&tempX_.vector,0,delta); //delta_p = inv(Jac)*(q-dest)';

        this->p[2]-=gsl_vector_get(delta,0);
        this->p[3]-=gsl_vector_get(delta,1);
        this->p[4]-=gsl_vector_get(delta,2);
        this->length-=gsl_vector_get(delta,3);

        gsl_permutation_free(perm);
        gsl_matrix_free(invertJac);
        gsl_vector_free(delta);

        break;
    }
    }
}

void PathParameterArc::calculateJacobi(double *jacobi)
{
    switch(this->order)
    {
    case 2:
    {
        int N = POINTNUM_INTERNAL;
        int w;
        double s,f,g,theta,k;
        double x1=0,x2=0;
        double y1=0,y2=0;
        for(int i=0;i<=N;i++)
        {
            if(i==0 || i==N) w=1;
            else if((i%2)==1) w=4;
            else w=2;

            s=length*i/(N-1);
            theta = getBiasThetaByS(s);
            f=cos(theta);
            g=sin(theta);
            x1+=gsl_pow_2(s)*w*g;
            x2+=gsl_pow_3(s)*w*g;
            y1+=gsl_pow_2(s)*w*f;
            y2+=gsl_pow_3(s)*w*f;
        }
        x1*=length/(3*N);  x2*=length/(3*N);
        y1*=length/(3*N);   y2*=length/(3*N);

        theta = getBiasThetaByS(length);
        k = getCurvatureByS(length);

        jacobi[0]=-x1/2;                jacobi[1]=-x2/3;                jacobi[2]=cos(theta);
        jacobi[3]= y1/2;                jacobi[4]= y2/3;                jacobi[5]=sin(theta);
        jacobi[6]=gsl_pow_2(length)/2;  jacobi[7]=gsl_pow_3(length)/3;  jacobi[8]=k;
        break;
    }
    case 3:
    {
        int N = POINTNUM_INTERNAL;
        int w;
        double s,f,g,theta,k;
        double x1=0,x2=0,x3=0;
        double y1=0,y2=0,y3=0;
        for(int i=0;i<=N;i++){
            if(i==0 || i==N) w=1;
            else if((i%2)==1) w=4;
            else w=2;

            s=this->length*i/(N-1);
            theta = this->getBiasThetaByS(s); /*thetaFunction(PathParameterArc::CUBIC, p, s)*/;
            f=cos(theta);
            g=sin(theta);
            x1+=gsl_pow_2(s)*w*g;
            x2+=gsl_pow_3(s)*w*g;
            x3+=gsl_pow_4(s)*w*g;
            y1+=gsl_pow_2(s)*w*f;
            y2+=gsl_pow_3(s)*w*f;
            y3+=gsl_pow_4(s)*w*f;
        }
        x1*=this->length/(3*N);  x2*=this->length/(3*N);  x3*=this->length/(3*N);
        y1*=this->length/(3*N);   y2*=this->length/(3*N);  y3*=this->length/(3*N);

        theta = this->getBiasThetaByS(this->length); /*thetaFunction(PathParameterArc::CUBIC, p, this->length)*/;
        k = this->getCurvatureByS(this->length); /*curvatureFunction(PathParameterArc::CUBIC, p, this->length)*/;

        jacobi[0]=-x1/2;             jacobi[1]=-x2/3;            jacobi[2]=-x3/4;               jacobi[3]=cos(theta);
        jacobi[4]=y1/2;              jacobi[5]=y2/3;             jacobi[6]=y3/4;                jacobi[7]=sin(theta);
        jacobi[8]=gsl_pow_2(this->length)/2;  jacobi[9]=gsl_pow_3(this->length)/3; jacobi[10]=gsl_pow_4(this->length)/4;   jacobi[11]=k;
        jacobi[12]=this->length;              jacobi[13]=gsl_pow_2(this->length);  jacobi[14]=gsl_pow_3(this->length);     jacobi[15]=this->p[1]+2*this->p[2]*this->length+3*this->p[3]*this->length*this->length;

        break;
    }
    case 4:
    {
        int N = POINTNUM_INTERNAL;
        int w;
        double s,f,g,theta,k,dk;
        double x1=0,x2=0,x3=0,x4=0;
        double y1=0,y2=0,y3=0,y4=0;
        for(int i=0;i<=N;i++){
            if(i==0 || i==N) w=1;
            else if((i%2)==1) w=4;
            else w=2;

            s= length*i/(N-1);
            theta =  getBiasThetaByS(s); /*thetaFunction(PathParameterArc::QUARTIC, p, s)*/;
            f=cos(theta);
            g=sin(theta);
            x1+=gsl_pow_2(s)*w*g;
            x2+=gsl_pow_3(s)*w*g;
            x3+=gsl_pow_4(s)*w*g;
            x4+=gsl_pow_5(s)*w*g;

            y1+=gsl_pow_2(s)*w*f;
            y2+=gsl_pow_3(s)*w*f;
            y3+=gsl_pow_4(s)*w*f;
            y4+=gsl_pow_5(s)*w*f;
        }
        x1*= length/(3*N);  x2*= length/(3*N);  x3*= length/(3*N); x4*= length/(3*N);
        y1*= length/(3*N);   y2*= length/(3*N);  y3*= length/(3*N); y4*= length/(3*N);

        theta =  getBiasThetaByS( length);
        k =  getCurvatureByS( length);
        dk =  getDCurvatureByS( length);

        jacobi[0]=-x2/3;                        jacobi[1]=-x3/4;            jacobi[2]=-x4/5;               jacobi[3]=cos(theta);
        jacobi[4]=y2/3;                         jacobi[5]=y3/4;             jacobi[6]=y4/5;                jacobi[7]=sin(theta);
        jacobi[8]=gsl_pow_3( length)/3;    jacobi[9]=gsl_pow_4( length)/4; jacobi[10]=gsl_pow_5( length)/5;   jacobi[11]=k;
        jacobi[12]=gsl_pow_2( length);     jacobi[13]=gsl_pow_3( length);  jacobi[14]=gsl_pow_4( length);     jacobi[15]=dk;

        break;
    }
    }

}

bool PathParameterArc::isTermination(StateSpatial currTransformedEndState, StateSpatial goalTransformedEndState)
{
    switch(this->order)
    {
    case 2:
    {
        if( (fabs(currTransformedEndState.x-goalTransformedEndState.x) <= 0.001) &&
                (fabs(currTransformedEndState.y-goalTransformedEndState.y) <= 0.001) &&
                (fabs(fmod(currTransformedEndState.theta-goalTransformedEndState.theta,2*M_PI)) <= 0.01) )
        {
            return true;
        }
        else
        {
            return false;
        }
        break;
    }
    case 3:
    {
        if( (fabs(currTransformedEndState.x-goalTransformedEndState.x) <= 0.001) &&
                (fabs(currTransformedEndState.y-goalTransformedEndState.y) <= 0.001) &&
                (fabs(fmod(currTransformedEndState.theta-goalTransformedEndState.theta,2*M_PI)) <= 0.01) &&
                (fabs(currTransformedEndState.k-goalTransformedEndState.k) <= 0.005) )
        {
            return true;
        }
        else
        {
            return false;
        }
        break;
    }
    case 4:
    {
        if( (fabs(currTransformedEndState.x-goalTransformedEndState.x) <= 0.001) &&
                (fabs(currTransformedEndState.y-goalTransformedEndState.y) <= 0.001) &&
                (fabs(fmod(currTransformedEndState.theta-goalTransformedEndState.theta,2*M_PI)) <= 0.01) &&
                (fabs(currTransformedEndState.k-goalTransformedEndState.k) <= 0.005) )
        {
            return true;
        }
        else
        {
            return false;
        }
        break;
    }
    }
}

std::pair<double,double> PathParameterArc::getXYByS(double s)
{
    int N = POINTNUM_INTERNAL;
    double sk, k, dk, x, y, dx, dy, dx_1, dy_1, theta, theta_1;
    dx_1=0;
    dy_1=0;
    theta_1=0;
    for(int i = 0; i < N; i++)
    {
        sk = i*s/(N-1);
        k = getCurvatureByS(sk);
        dk = getDCurvatureByS(sk);
        if(i==0)
        {
            x=0;
            y=0;
            theta=0;
            dx_1=0;
            dy_1=0;
            theta_1=theta;
        }
        else
        {
            theta = getBiasThetaByS(sk);
            dx=dx_1 * (i-1) / i + (cos(theta) + cos(theta_1)) / (2*i);
            dy=dy_1 * (i-1) / i + (sin(theta) + sin(theta_1)) / (2*i);
            x=sk*dx;
            y=sk*dy;

            dx_1=dx;
            dy_1=dy;
            theta_1=theta;
        }
    }

    std::pair<double,double> tmp;
    tmp.first = s0.x + cos(s0.theta) * x - sin(s0.theta) * y;
    tmp.second = s0.y + cos(s0.theta) * y + sin(s0.theta) * x;

    return tmp;
}

std::pair<double,double> PathParameterArc::getXYByS_Cheap(double s)
{
    int idx = getIdOnHelpingPathByS(s);

    std::pair<double,double> tmp( helpingPath.x[idx], helpingPath.y[idx] );
    return tmp;
}

double PathParameterArc::getBiasThetaByS(double s)
{
    double theta = 0.0;

    switch(this->order)
    {
    case 2:
    {
        double c[4]={0.0, p[0], p[1]/2, p[2]/3};
        theta=gsl_poly_eval(c,4,s);
        break;
    }
    case 3:
    {
        double c[5]={0.0, p[0], p[1]/2, p[2]/3, p[3]/4};
        theta=gsl_poly_eval(c,5,s);
        break;
    }
    case 4:
    {
        double c[6]={0.0, p[0], p[1]/2, p[2]/3, p[3]/4, p[4]/5};
        theta=gsl_poly_eval(c,6,s);
        break;
    }
    }

    return theta;
}

double PathParameterArc::getCurvatureByS(double s)
{
    double k = 0.0;

    switch(this->order)
    {
    case 2:
    {
        double c[3]={p[0], p[1], p[2]};
        k=gsl_poly_eval(c,3,s);
        break;
    }
    case 3:
    {
        double c[4]={p[0], p[1], p[2], p[3]};
        k=gsl_poly_eval(c,4,s);
        break;
    }
    case 4:
    {
        double c[5]={p[0], p[1], p[2], p[3], p[4]};
        k=gsl_poly_eval(c,5,s);
        break;
    }
    }

    return k;
}

double PathParameterArc::getDCurvatureByS(double s)
{
    double dk = 0.0;

    switch(this->order)
    {
    case 2:
    {
        double c[2]={p[1], 2 * p[2]};
        dk=gsl_poly_eval(c,2,s);
        break;
    }
    case 3:
    {
        double c[3]={p[1], 2 * p[2], 3 * p[3]};
        dk=gsl_poly_eval(c,3,s);
        break;
    }
    case 4:
    {
        double c[4]={p[1], 2*p[2], 3*p[3], 4*p[4]};
        dk=gsl_poly_eval(c,4,s);
        break;
    }
    }

    return dk;
}

double PathParameterArc::getDDCurvatureByS(double s)
{
    double ddk = 0.0;

    switch(this->order)
    {
    case 2:
    {
        double c[1]={2 * p[2]};
        ddk=gsl_poly_eval(c,1,s);
        break;
    }
    case 3:
    {
        double c[2]={2 * p[2], 6 * p[3]};
        ddk=gsl_poly_eval(c,2,s);
        break;
    }
    case 4:
    {
        double c[3]={2*p[2], 6*p[3], 12*p[4]};
        ddk=gsl_poly_eval(c,3,s);
        break;
    }
    }

    return ddk;
}

Path PathParameterArc::plotPath(int pointNum)
{
    Path tmpPath;

    if( ! this->isValid ) {
        tmpPath.isValid = false;
        return tmpPath;
    }

    tmpPath.resize(pointNum);

    int N = pointNum;
    double sk, k, dk, x, y, dx, dy, dx_1, dy_1, theta, theta_1;
    dx_1=0;
    dy_1=0;
    theta_1=0;
    for(int i=0;i<N;i++)
    {
        sk = i*length/(N-1);
        k = getCurvatureByS(sk);
        dk = getDCurvatureByS(sk);
        if(i==0)
        {
            x=0;
            y=0;
            theta=0;
            dx_1=0;
            dy_1=0;
            theta_1=theta;
        }
        else
        {
            theta = getBiasThetaByS(sk);
            dx=dx_1 * (i-1) / i + (cos(theta) + cos(theta_1)) / (2*i);
            dy=dy_1 * (i-1) / i + (sin(theta) + sin(theta_1)) / (2*i);
            x=sk*dx;
            y=sk*dy;

            dx_1=dx;
            dy_1=dy;
            theta_1=theta;
        }


        tmpPath.s[i] = sk;
        tmpPath.x[i] = s0.x + cos(s0.theta) * x - sin(s0.theta) * y;
        tmpPath.y[i] = s0.y + cos(s0.theta) * y + sin(s0.theta) * x;

        //theta
        tmpPath.theta[i] = (theta + s0.theta);
        double theta1,theta2;
        theta1 = fmod(tmpPath.theta[i]+8*M_PI,2*M_PI);
        theta2 = theta1-2*M_PI;
        if(fabs(theta1)<=fabs(theta2)) { tmpPath.theta[i] = theta1; }
        else { tmpPath.theta[i] = theta2; }

        tmpPath.k[i] = k;
        tmpPath.dk[i] = dk;
    }
    tmpPath.isValid = true;


    return tmpPath;
}

int PathParameterArc::getIdOnHelpingPathByS(double s)
{
    int minId = -1;
    double minDistS = GSL_POSINF;
    for(int ii=0; ii<helpingPath.numOfPoints; ii++)
    {
        double distS = fabs(helpingPath.s[ii] - s);
        if(distS < minDistS)
        {
            minDistS = distS;
            minId = ii;
        }
    }

    return minId;
}

/************************************ DEFINITIONs for struct VelocityParameterTime **************************************/
void VelocityParameterTime::reset()
{
    s0.reset();
    sf.reset();

    p[0] = 0.0; p[1] = 0.0; p[2] = 0.0; p[3] = 0.0; p[4] = 0.0;
    duration = 0.0;

    order = 1;
    isValid = false;
}

void VelocityParameterTime::findVelocityParameter(StateTemporal startState, double acc, PathParameterArc* pathPar)
{
    s0 = startState;
    order = 1;

    double pathLength_m = pathPar->length;

    if( pathLength_m < 1E-1 ) {
        sf.v = 0.0;
        sf.a = 0.0;
        p[0] = 0.0;
        p[1] = 0.0;
        p[2] = 0.0;
        p[3] = 0.0;
        p[4] = 0.0;
        duration = 0.0;
        isValid = true;
        return;
    }

    double insideSqrt = s0.v*s0.v + 2*acc*pathLength_m;

    if( insideSqrt > 1E-2 ) {
        sf.v = sqrt( insideSqrt );
        sf.a = 0.0;
        if( fabs(acc) >= 1E-2 ) {
            p[0] = s0.v;
            p[1] = acc;
            p[2] = 0.0;
            p[3] = 0.0;
            p[4] = 0.0;
            duration = fabs( (sf.v-s0.v) / acc );
            isValid = true;
        }
        else if( fabs(sf.v >= 1E-2) ) {
            p[0] = s0.v;
            p[1] = acc;
            p[2] = 0.0;
            p[3] = 0.0;
            p[4] = 0.0;
            duration = fabs( pathLength_m / sf.v );
            isValid = true;
        }
        else {
            p[0] = 0.0;
            p[1] = 0.0;
            p[2] = 0.0;
            p[3] = 0.0;
            p[4] = 0.0;
            duration = 0.0;
            isValid = false;
        }
    }
    else {
        if( fabs(acc) >= 1E-2 ) {
            p[0] = s0.v;
            p[1] = acc;
            p[2] = 0.0;
            p[3] = 0.0;
            p[4] = 0.0;
            duration = fabs( s0.v / acc );
            isValid = true;
        }
        else {
            p[0] = 0.0;
            p[1] = 0.0;
            p[2] = 0.0;
            p[3] = 0.0;
            p[4] = 0.0;
            duration = 0.0;
            isValid = false;
        }
    }

    pathPar->length = (s0.v + sf.v)/2 * duration;
}

void VelocityParameterTime::findVelocityParameter(StateTemporal startState, StateTemporal endState, PathParameterArc* pathPar, int order)
{
    this->s0 = startState;
    this->sf = endState;
    this->order = order;

    if(order == 3) {
        double v0 = startState.v;
        double a0 = startState.a;
        double vf = endState.v;
        double sf = pathPar->length;

        if( (fabs(sf) <= 1e-6) || (fabs(a0) <= 1e-6 && fabs(v0+vf) <= 1e-6) ) {
            //the path is a point
            this->p[0] = 0; this->p[1] = 0; this->p[2] = 0; this->p[3] = 0; this->p[4] = 0;
            this->duration = 0;
            this->isValid = false;
            return;
        }

        double tf = 0;
        if ( fabs(a0) <= 1e-6 ) {
            tf = 2 * sf / ( v0 + vf );
        }
        else {
            double insideSqure = ((v0+vf)/2) * ((v0+vf)/2) + a0*sf/3;
            if ( insideSqure < 0) {
                this->isValid = false;
                return;
            }
            else { tf = (- (v0+vf)/2 + sqrt( fabs(insideSqure) ) ) / ( a0/6 ); }
        }

        if( tf < 0.01 ) {
            this->p[0] = 0.0;
            this->p[1] = 0.0;
            this->p[2] = 0.0;
            this->p[3] = 0.0;
            this->p[4] = 0.0;
            this->duration = 0.0;
        }
        else {
            this->p[0] = v0;
            this->p[1] = a0;
            this->p[2] = ( 3*(vf-v0) - 2*a0*tf ) / ( tf * tf );
            this->p[3] = ( 2*(v0-vf) +   a0*tf ) / ( tf * tf * tf );
            this->p[4] = 0;
            this->duration = tf;
        }
        this->order = order;
        this->isValid = true;
    }
    else {
        double v0 = startState.v;
        double vf = endState.v;

        double sf = pathPar->length;

        if(fabs(sf) <= 1e-6 || fabs(v0+vf) <= 1e-6) {
            //the path is a point
            this->isValid = false;
            return;
        }

        double tf = 2 * sf / ( v0 + vf );

        if( tf < 0.01 ) {
            this->p[0] = 0.0;
            this->p[1] = 0.0;
            this->p[2] = 0.0;
            this->p[3] = 0.0;
            this->p[4] = 0.0;
            this->duration = 0.0;
        }
        else {
            this->p[0] = v0;
            this->p[1] = (vf-v0)/tf;
            this->p[2] = 0;
            this->p[3] = 0;
            this->p[4] = 0;
            this->duration = tf;
        }
        this->order = order;
        this->isValid = true;
    }

}

double VelocityParameterTime::getSByT(double t)
{
    double c[5]={0.0, this->p[0], this->p[1]/2, this->p[2]/3, this->p[3]/4};
    return gsl_poly_eval(c,5,t);
}

double VelocityParameterTime::getVByT(double t)
{
    double c[4]={this->p[0], this->p[1], this->p[2], this->p[3]};
    return gsl_poly_eval(c,4,t);
}

double VelocityParameterTime::getAByT(double t)
{
    double c[4]={this->p[1], 2 * this->p[2], 3 * this->p[3]};
    return gsl_poly_eval(c,3,t);
}


Velocity VelocityParameterTime::plotVelocity(int pointNum)
{
    Velocity vel;

    vel.resize(pointNum);

    double tk;
    double dt = duration /  (pointNum - 1);

    for(int i=0 ; i < pointNum ; i++)
    {
        tk = i * dt;
        vel.t[i] = tk;
        vel.s[i] = getSByT(tk);
        vel.v[i] = getVByT(tk);
        vel.a[i] = getAByT(tk);
    }

    return vel;
}


/************************************ DEFINITIONs for struct TRAJECTORY **************************************/
void Trajectory::reset()
{
//    s0.reset();
//    sf.reset();

    pathPar.reset();
    velPar.reset();

    timeDuration_s = 0.0;
    numOfPoints = 0;
    cost = 0.0;

    isValid = false;
}


void Trajectory::clearPoints(int num)
{
    t.clear();          t.resize(num);
    x_t.clear();        x_t.resize(num);
    y_t.clear();        y_t.resize(num);
    theta_t.clear();    theta_t.resize(num);
    k_t.clear();        k_t.resize(num);
    dk_t.clear();        dk_t.resize(num);

    s_t.clear();        s_t.resize(num);
    v_t.clear();        v_t.resize(num);
    a_t.clear();        a_t.resize(num);

    numOfPoints = num;
}


void Trajectory::evaluateTrajectoryGivenDeltaTime(double dt)
{
    if( !pathPar.isValid || !velPar.isValid ) {
        numOfPoints = 0;
        isValid = false;
        return;
    }

    // numerically construct trajectory
    timeDuration_s = velPar.duration;
    int pointNum = floor(timeDuration_s / dt);

    clearPoints(pointNum);

    double tk, sk;
    for(int ii = 0 ; ii < pointNum ; ii++)
    {
        tk = ii * dt;
        t[ii] = tk;
        v_t[ii] = velPar.getVByT(tk);
        a_t[ii] = velPar.getAByT(tk);

        sk = velPar.getSByT(tk);
        s_t[ii] = sk;
        std::pair<double, double> xy = pathPar.getXYByS_Cheap(sk);
        x_t[ii] =  xy.first;
        y_t[ii] =  xy.second;
        theta_t[ii] = pathPar.s0.theta + pathPar.getBiasThetaByS(sk);
        k_t[ii] = pathPar.getCurvatureByS(sk);
        dk_t[ii] = pathPar.getDCurvatureByS(sk);
    }

    isValid = true;

    return;
}

void Trajectory::evaluateTrajectoryGivenPointNum(int pointNum)
{
    if( !pathPar.isValid || !velPar.isValid ) {
        numOfPoints = 0;
        isValid = false;
        return;
    }

    // numerically construct trajectory
    clearPoints(pointNum);

    timeDuration_s = velPar.duration;
    double tk, sk;
    double dt = timeDuration_s /  (pointNum - 1);

    for(int ii = 0 ; ii < pointNum ; ii++) {
        tk = ii * dt;
        t[ii] = tk;
        v_t[ii] = velPar.getVByT(tk);
        a_t[ii] = velPar.getAByT(tk);

        sk = velPar.getSByT(tk);
        s_t[ii] = sk;
        std::pair<double, double> xy = pathPar.getXYByS_Cheap(sk);
        x_t[ii] =  xy.first;
        y_t[ii] =  xy.second;
        theta_t[ii] = pathPar.s0.theta + pathPar.getBiasThetaByS(sk);
        k_t[ii] = pathPar.getCurvatureByS(sk);
        dk_t[ii] = pathPar.getDCurvatureByS(sk);

//        double k = k_t[ii];
//        double dk_dt = pathPar.getDCurvatureByS(sk) * v_t[ii];
//        double ddk_ddt = pathPar.getDDCurvatureByS(sk) * v_t[ii] + pathPar.getDCurvatureByS(sk) * a_t[ii];

//        double phi = atan(VEH_WHEELBASE * k);
//        double dphi_dt = VEH_WHEELBASE * dk_dt * cos(phi) * cos(phi);
//        double ddphi_ddt = VEH_WHEELBASE * ddk_ddt * cos(phi) * cos(phi) - 2 * dphi_dt * dphi_dt * tan(phi);

//        if(fabs(phi) >= PHI_LIMIT || fabs(dphi_dt) >= DPHI_LIMIT || fabs(ddphi_ddt) >= DDPHI_LIMIT)
//        {
//            isValid = false;
//            return;
//        }
    }

    // Update the path length
    this->pathPar.length = velPar.getSByT(timeDuration_s);

    isValid = true;

    return;

}

void Trajectory::setTrajectoryParameters(PathParameterArc pathPar, VelocityParameterTime velPar)
{
//    this->timeOffset_s = offsetTime;
    this->pathPar = pathPar;
    this->velPar = velPar;
}

/**
 * @brief Trajectory Generation
 *
 */
void Trajectory::generateOneTrajectory(State startState, State endState, int pathOrder, int velOrder, int pointNum)
{
//    timeOffset_s = offsetTime;

    pathPar.findPathParameter(startState.spatial, endState.spatial, pathOrder);
    velPar.findVelocityParameter(startState.temporal, endState.temporal, &pathPar, velOrder);
    evaluateTrajectoryGivenPointNum(pointNum);
}

void Trajectory::generateOneTrajectoryConstAcc(State startState, State endState, double acc, int pathOrder, int pointNum)
{
//    timeOffset_s = offsetTime;

    pathPar.findPathParameter(startState.spatial, endState.spatial, pathOrder);
    velPar.findVelocityParameter(startState.temporal, acc, &pathPar );
    evaluateTrajectoryGivenPointNum(pointNum);
}

void Trajectory::generateOneTrajectoryConstDT(State startState, State endState, int pathOrder, int velOrder, double dt)
{
//    timeOffset_s = offsetTime;

    pathPar.findPathParameter(startState.spatial, endState.spatial, pathOrder);
    velPar.findVelocityParameter(startState.temporal, endState.temporal, &pathPar, velOrder);
    evaluateTrajectoryGivenDeltaTime(dt);
}

void Trajectory::generateOneTrajectoryStraightLine(State startState, State endState, int pointNum)
{
//    timeOffset_s = offsetTime;

    pathPar.reset();
    pathPar.s0.x = startState.spatial.x;
    pathPar.s0.y = startState.spatial.y;
    pathPar.s0.theta = atan2( endState.spatial.y - startState.spatial.y,
                              endState.spatial.x - startState.spatial.x );
    pathPar.length = hypot( startState.spatial.x - endState.spatial.x,
                            startState.spatial.y - endState.spatial.y );
    pathPar.isValid = true;


    velPar.reset();
    velPar.s0 = startState.temporal;
    velPar.sf = endState.temporal;

    double v0 = velPar.s0.v;
    double vf = velPar.sf.v;
    double sf = pathPar.length;

    if(fabs(sf) <= 1e-6 || fabs(v0+vf) <= 1e-6) {
        //the path is a point
        velPar.p[0] = 0.0;
        velPar.p[1] = 0.0;
        velPar.p[2] = 0.0;
        velPar.p[3] = 0.0;
        velPar.p[4] = 0.0;
        velPar.duration = 0.0;
    }
    else {
        double tf = 2 * sf / ( v0 + vf );

        velPar.p[0] = v0;
        velPar.p[1] = (vf-v0)/tf;
        velPar.p[2] = 0;
        velPar.p[3] = 0;
        velPar.p[4] = 0;
        velPar.duration = tf;
    }
    velPar.order = 1;
    velPar.isValid = true;


    evaluateTrajectoryGivenPointNum(pointNum);

    return;
}

int Trajectory::getProjIdxByT(double t)
{
    int minId = -1;
    double minTDist = GSL_POSINF;

    for(int ii=0; ii<numOfPoints; ii++)
    {
        double TDist = fabs(t - this->t[ii]);
        if(TDist < minTDist)
        {
            minTDist = TDist;
            minId = ii;
        }
    }

    return minId;
}

int Trajectory::getProjIdxByS(double s)
{
    int minId = -1;
    double minSDist = GSL_POSINF;

    for(int ii=0; ii<numOfPoints; ii++)
    {
        double SDist = fabs(s - this->s_t[ii]);
        if(SDist < minSDist)
        {
            minSDist = SDist;
            minId = ii;
        }
    }

    return minId;
}

int Trajectory::getProjIdxByXY(double x, double y)
{
    int minId = -1;
    double minDist = GSL_POSINF;

//    cout << "traj.numOfPoints = " << this->numOfPoints << endl;
//    cout << "traj.x_t.size() = " << this->x_t.size() << endl;
//    cout << "traj.y_t.size() = " << this->y_t.size() << endl;

    for(int ii=0; ii<numOfPoints; ii++) {
        double dist = hypot(x - this->x_t[ii], y - this->y_t[ii]);
        if(dist < minDist) {
            minDist = dist;
            minId = ii;
        }
    }

    return minId;
}

//void TRAJECTORY::evaluateTrajectory_Model(double dt)
//{
//    // find path parameter
////    pathPar.findPathParameter(startState.spatial, endState.spatial, pathOrder);
////    if( !pathPar.isValid ) { isValid = false; }
////    // find speed parameter
////    velPar.findVelocityParameter(startState.temporal, endState.temporal, &pathPar, velOrder);
////    if( !velPar.isValid ) { isValid = false; }

////    // numerically construct trajectory
////    totalT = velPar.duration;
////    isValid = true;

//    int num = (int)floor( totalT / dt + 1 );

//    double t_t[num];
//    double v_t[num];
//    double a_t[num];
//    double s_t[num];

//    double x_t[num];
//    double y_t[num];
//    double theta_t[num];
//    double k_t[num];
//    double dk_t[num];


//    for(int ii = 0 ; ii < num ; ii++)
//    {
//        t_t[ii] = ii * dt;
//        v_t[ii] = velPar.getVByT( t_t[ii] );
//        a_t[ii] = velPar.getAByT( t_t[ii] );

//        double sk = velPar.getSByT( t_t[ii] );
//        s_t[ii] = sk;
//        std::pair<double, double> xy = pathPar.getXYByS(sk);
//        x_t[ii] =  xy.first;
//        y_t[ii] =  xy.second;
//        theta_t[ii] = pathPar.s0.theta + pathPar.getBiasThetaByS(sk);
//        k_t[ii] = pathPar.getCurvatureByS(sk);
//        dk_t[ii] = pathPar.getDCurvatureByS(sk);
//    }

//    // Obtain delta_t, ddelta_t, dddelta_t
//    double delta_t[num], ddelta_t[num], dddelta_t[num];

//    for (int i = 0; i < num ; ++i)
//    {
//        delta_t[i] = atan(2.8 * k_t[i]);
//    }

//    for (int i=0; i<num; i++)
//    {
//        if(i == num-1 && i != 0)
//            ddelta_t[i] = ddelta_t[i-1];
//        else
//            ddelta_t[i] = (delta_t[i+1] - delta_t[i]) / (t_t[i+1] - t_t[i]);
//    }
//    for (int i=0; i<num; i++)
//    {
//        if(i == num-1 && i != 0)
//            dddelta_t[i] = dddelta_t[i-1];
//        else
//            dddelta_t[i] = (ddelta_t[i+1] - ddelta_t[i]) / (t_t[i+1] - t_t[i]);
//    }

//    // Run through model
//    double X0_Jarrod_DDDelta[6];
//    X0_Jarrod_DDDelta[0] = x_t[0];
//    X0_Jarrod_DDDelta[1] = y_t[0];
//    X0_Jarrod_DDDelta[2] = theta_t[0];
//    X0_Jarrod_DDDelta[3] = delta_t[0];
//    X0_Jarrod_DDDelta[4] = ddelta_t[0];
//    X0_Jarrod_DDDelta[5] = v_t[0];


//    vector<double> t_traj, x_traj, y_traj, k_traj, v_traj;
//    t_traj.resize(num, 0.0);
//    x_traj.resize(num, 0.0);
//    y_traj.resize(num, 0.0);
//    k_traj.resize(num, 0.0);
//    v_traj.resize(num, 0.0);

//    EvaluateTraj_Jarrod_DDDelta(X0_Jarrod_DDDelta, t_t, dddelta_t, a_t, num, dt,
//                                t_traj, x_traj, y_traj, k_traj, v_traj);

//    this->x_traj = x_traj;
//    this->y_traj = y_traj;

//    pathParModel.reset();
//    pathParModel.s0 = pathPar.s0;
//    pathParModel.duration = totalT;
//    pathParModel.order = 6;
//    pathParModel.isValid = true;

//    Fitting fitter;
//    std::vector<double> P;
//    fitter.findPathParameter(t_traj, k_traj, P);
//    // Use an optimizaiton routine to fit the parameters between t_traj -- k_traj
//    task::Task::get()->getLogger().log_debug("Fitted Parameters p0[%f] p1[%f] p2[%f] p3[%f] p4[%f] p5[%f]",
//                                             P[0], P[1], P[2], P[3], P[4], P[5]);

//////    velParModel.reset();
////    velParModel.s0 = startState.temporal;
////    velParModel.duration = totalT;
////    velParModel.order = 5;
////    velParModel.isValid = true;
//    // Use an optimizaiton routine to fit the parameters between t_traj -- v_traj
//}

//void TRAJECTORY::EvaluateTraj_Jarrod_DDDelta(double* X0, double* t_t, double* dddelta_t, double* a_t, int Tnum, double dt,
//                                             vector<double>& t_traj, vector<double>& x_traj, vector<double>& y_traj, vector<double>& k_traj, vector<double>& v_traj)
//{
//    // P[0]: Gss
//    // P[1]: L
//    double P[2];
//    P[0] = 1.0;
//    P[1] = 2.8;

//    // C[0]: dddelta_max
//    // C[1]: ddelta_max
//    // C[2]: a_min
//    // C[3]: a_max
//    // C[4]: v_min
//    // C[5]: v_max
//    double C[6];
//    C[0] = 3.0;
//    C[1] = 0.1;
//    C[2] = -10.0;
//    C[3] = 4.0;
//    C[4] = 0.0;
//    C[5] = 50.0;

//    // X[0]: x
//    // X[1]: y
//    // X[2]: theta
//    // X[3]: delta
//    // X[4]: ddelta
//    // X[5]: v
//    double X[6];
//    X[0] = X0[0];
//    X[1] = X0[1];
//    X[2] = X0[2];
//    X[3] = X0[3];
//    X[4] = X0[4];
//    X[5] = X0[5];

//    t_traj[0] = 0.0;
//    x_traj[0] = X[0];
//    y_traj[0] = X[1];
//    k_traj[0] = tan(X[3]) / 2.8;
//    v_traj[0] = X[5];

//    for(int i = 1; i < Tnum; i++)
//    {
//        double I[2];
//        I[0] = dddelta_t[i-1];
//        I[1] = a_t[i-1];

//        // calculate the diff
//        double dX[6];
//        Diff_Jarrod_DDDelta(I, X, P, C,
//                            dX);

//        X[0] += dt * dX[0];
//        X[1] += dt * dX[1];
//        X[2] += dt * dX[2];
//        X[3] += dt * dX[3];
//        X[4] += dt * dX[4];
//        X[5] += dt * dX[5];


//        t_traj[i] = i * dt;
//        x_traj[i] = X[0];
//        y_traj[i] = X[1];
//        k_traj[i] = tan(X[3]) / 2.8;
//        v_traj[i] = X[5];
//    }
//}

//void TRAJECTORY::Diff_Jarrod_DDDelta(double* I, double* X, double* P, double* C,
//                                     double* dX)
//{
//    // I[0]: cmdDDDelta
//    // I[1]: cmdA

//    // X[0]: x
//    // X[1]: y
//    // X[2]: theta
//    // X[3]: delta
//    // X[4]: ddelta
//    // X[5]: v

//    // P[0]: Gss
//    // P[1]: L

//    // C[0]: dddelta_max
//    // C[1]: ddelta_max
//    // C[2]: a_min
//    // C[3]: a_max
//    // C[4]: v_min
//    // C[5]: v_max

//    if (I[0] > C[0]) { I[0] = C[0]; }
//    else if (I[0] < -C[0]) { I[0] = -C[0]; }

//    if (I[1] > C[3]) { I[1] = C[3]; }
//    else if (I[1] < C[2]) { I[1] = C[2]; }

//    if (X[4] > C[1]) { X[4] = C[1]; }
//    else if (X[4] < -C[1]) { X[4] = -C[1]; }

//    if (X[5] > C[5]) { X[5] = C[5]; }
//    else if (X[5] < C[4]) { X[5] = C[4]; }


//    dX[0] = X[5] * cos(X[2]);
//    dX[1] = X[5] * sin(X[2]);
//    dX[2] = P[0] * X[5] / P[1] * tan(X[3]);
//    dX[3] = X[4];
//    dX[4] = I[0];
//    dX[5] = I[1];
//}



void CTRLTRAJ::reset()
{
    s0.reset();
    dkdt = 0.0;
    dvdt = 0.0;
    totalT = 0.0;

    numOfPoints = 0;
}

void CTRLTRAJ::clearPoints(int num)
{
    t.clear();          t.resize(num);
    s_t.clear();        s_t.resize(num);
    x_t.clear();        x_t.resize(num);
    y_t.clear();        y_t.resize(num);
    theta_t.clear();    theta_t.resize(num);
    k_t.clear();        k_t.resize(num);
    v_t.clear();        v_t.resize(num);

    numOfPoints = num;
}

void CTRLTRAJ::generateOneTrajectory(State s0, double dkdt, double dvdt, double totalT)
{
    this->s0 = s0;
    this->dkdt = dkdt;
    this->dvdt = dvdt;
    this->totalT = totalT;

    double dt = 0.1;

    double Vmin(0.0), Vmax(40.0);

    this->numOfPoints = floor(totalT / dt);

    clearPoints(numOfPoints);

    double s(0.0), s_1(0.0);
    double x(0.0), y(0.0), h(0.0), k(0.0);

    double xl(0.0), xl_1(0.0);
    double yl(0.0), yl_1(0.0);
    double thetal(0.0), thetal_1(0.0);

    double v(s0.temporal.v), v_1(0.0);

    for(int ii = 0; ii < this->numOfPoints; ii++)
    {
        if(ii == 0)
        {
            t[ii] = ii * dt;

            s_t[ii] = 0.0;
            v_t[ii] = s0.temporal.v;

            x_t[ii] = s0.spatial.x;
            y_t[ii] = s0.spatial.y;
            theta_t[ii] = s0.spatial.theta;
            k_t[ii] = s0.spatial.k;
        }
        else
        {
            v_1 = v;
            v = v + dvdt * dt;

            v = max(v, Vmin);
            v = min(v, Vmax);

            s_1 = s;
            s = s + (v_1+v)/2 * dt;

            thetal_1 = thetal;
            thetal = thetaFunctionByT(s0.spatial.k, dkdt, ii * dt);

            xl_1 = xl; yl_1 = yl;
            xl = xl_1 + (cos(thetal)+cos(thetal_1)) / 2 * (s-s_1);
            yl = yl_1 + (sin(thetal)+sin(thetal_1)) / 2 * (s-s_1);

            x = s0.spatial.x + cos(s0.spatial.theta) * xl - sin(s0.spatial.theta) * yl;
            y = s0.spatial.y + cos(s0.spatial.theta) * yl + sin(s0.spatial.theta) * xl;
            h = s0.spatial.theta + thetal;
            k = curvatureFunctionByT(s0.spatial.k, dkdt, ii * dt);

            t[ii] = ii * dt;

            s_t[ii] = s;
            v_t[ii] = v;

            x_t[ii] = x;
            y_t[ii] = y;
            theta_t[ii] = h;
            k_t[ii] = k;
        }

    }

}

double CTRLTRAJ::thetaFunctionByT(double k0, double dkdt, double tt)
{
    double c[3] = {0.0, k0, dkdt/2};
    return gsl_poly_eval(c, 3, tt);
}

double CTRLTRAJ::curvatureFunctionByT(double k0, double dkdt, double tt)
{
    double c[2] = {k0, dkdt};
    return gsl_poly_eval(c, 2, tt);
}
