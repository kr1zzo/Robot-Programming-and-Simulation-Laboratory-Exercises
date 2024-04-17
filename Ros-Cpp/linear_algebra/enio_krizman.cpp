#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/matrix_vector.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

int main() 
{
    using namespace boost::numeric::ublas;

    matrix<int> m1 (5, 5);
    matrix<int> m2 (5, 5);
    identity_matrix<int> m_ind (5,5);
    vector<int> v (5);
    vector<int> v_trans (5);
    matrix <double> inv_m2(5,5);
    int i;

    //a)
    //m1-each row contains first five digits of JMBAG
    for(unsigned i=0; i<m1.size1() ; i++){
        m1 (i,0) = 0;
        m1 (i,1) = 0;
        m1 (i,2) = 6;
        m1 (i,3) = 9;
        m1 (i,4) = 0;
    }
    std::cout<<"m1: "<<m1<<std::endl;

    //vector-last 5 digits of JBMAG
    v(0)=8;
    v(1)=3;
    v(2)=8;
    v(3)=4;
    v(4)=8;

    m2= m1+m_ind;

    std::cout<< "m2: "<<m2<<std::endl;

    std::cout<< "v: "<<v<<std::endl;

    //b)
    vector<int> result= prod(m2,v);
    
    std::cout<< "m2*v: "<<result<<std::endl;

    //c)
    v_trans = boost::numeric::ublas::trans(v);

    std::cout << "v trans: "<<v_trans << std::endl;

    std::cout<< "v*v_trans: "<<inner_prod(v,v_trans)<< std::endl;

    //d)
    std::cout << "m1+m2: "<< m1+m2 << std::endl;

    //e)
    matrix<double> m2_c(5, 5);
    m2_c =m2;

    permutation_matrix<double> pm(m2_c.size1());
    
    
    for (int i = 0; i < inv_m2.size1 (); i++)
        for (int j = 0; j < inv_m2.size2 (); j++)
            if(i==j) inv_m2(i,j) = 1;
    
    int res = lu_factorize(m2_c, pm);
    lu_substitute(m2_c, pm, inv_m2);
    
    std::cout <<"inv(m2): "<<inv_m2 << std::endl;      


    return 0;

}

