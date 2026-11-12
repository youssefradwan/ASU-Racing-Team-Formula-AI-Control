/*
Cubic Spline Interpolation Implementation
Burden and Faires - Numerical Analysis, 9th Edition
Chapter : 3, Section: 3.5
*/


#include "Cubic_Spline.h"


namespace mpc_controller{
    CubicSpline::CubicSpline() : spline_data_set_(false) {}

    void CubicSpline::setregularSpacing(const Eigen::VectorXd& x, const Eigen::VectorXd& y,const double spacing){
        
        if (x.size() == y.size()) 
        {
            splineData.x = x;
            splineData.y = y;
            splineData.n = x.size();
            splineData.is_evenly_spaced = true;
            splineData.spacing = spacing;
            spline_data_set_ = true;
        } 
        else 
        {
            std::cout << "Input vectors x and y must have the same size." << std::endl;
            //throw std::invalid_argument("Input vectors x and y must have the same size.");
        }
    }

    void CubicSpline::setirregularSpacing(const Eigen::VectorXd& x, const Eigen::VectorXd& y,const double spacing)
    {
        if (x.size() == y.size()) 
        {
            splineData.x = x;
            splineData.y = y;
            splineData.n = x.size();
            splineData.is_evenly_spaced = false;
            spline_data_set_ = true;
        } 
        else 
        {
            std::cout << "Input vectors x and y must have the same size." << std::endl;
            //throw std::invalid_argument("Input vectors x and y must have the same size.");
        }
    }

    bool CubicSpline::computeCoefficients() 
    {
        //compute the coefficients of the cubic spline based on the input data
        if (!spline_data_set_) {
            std::cout << "Data not set. Please set data before computing coefficients." << std::endl;
            return false;
        }

        const int n = splineData.n;

        params_.a.setZero(n);
        params_.b.setZero(n - 1);
        params_.c.setZero(n);
        params_.d.setZero(n - 1);

        params_.a = splineData.y;

        if (n < 3) {
            if (n == 2) {
                const double h0 = splineData.x(1) - splineData.x(0);
                if (h0 > 1e-12) params_.b(0) = (params_.a(1) - params_.a(0)) / h0;
            }
            return true;
        }

        // h[i] = x[i+1] - x[i]
        Eigen::VectorXd h(n - 1);
        for (int i = 0; i < n - 1; ++i)
            h(i) = splineData.x(i + 1) - splineData.x(i);

        // ── OPEN PATH: Natural boundary conditions ──────────────────────────────
        // For open (non-closed) paths (e.g. local hairpin path), use natural BCs:
        // c[0] = 0, c[n-1] = 0 (second derivative = 0 at endpoints).
        // This prevents the spline from creating artificial loops to connect
        // the start and end of the path, which is catastrophic for hairpins.
        if (!splineData.is_closed) {
            Eigen::VectorXd alpha = Eigen::VectorXd::Zero(n);
            for (int i = 1; i < n - 1; ++i) {
                alpha(i) = 3.0 / h(i) * (params_.a(i + 1) - params_.a(i))
                         - 3.0 / h(i - 1) * (params_.a(i) - params_.a(i - 1));
            }

            // Thomas algorithm for tridiagonal system
            Eigen::VectorXd l = Eigen::VectorXd::Ones(n);
            Eigen::VectorXd mu = Eigen::VectorXd::Zero(n);
            Eigen::VectorXd z = Eigen::VectorXd::Zero(n);

            for (int i = 1; i < n - 1; ++i) {
                l(i) = 2.0 * (splineData.x(i + 1) - splineData.x(i - 1)) - h(i - 1) * mu(i - 1);
                mu(i) = h(i) / l(i);
                z(i) = (alpha(i) - h(i - 1) * z(i - 1)) / l(i);
            }

            params_.c(n - 1) = 0.0;
            for (int j = n - 2; j >= 0; --j) {
                params_.c(j) = z(j) - mu(j) * params_.c(j + 1);
                params_.b(j) = (params_.a(j + 1) - params_.a(j)) / h(j)
                             - h(j) * (params_.c(j + 1) + 2.0 * params_.c(j)) / 3.0;
                params_.d(j) = (params_.c(j + 1) - params_.c(j)) / (3.0 * h(j));
            }
            return true;
        }

        // ── CLOSED PATH: Periodic boundary conditions ────────────────────────────
        // For closed tracks, periodic BCs ensure c[n-1] = c[0] and continuous
        // first derivative across the seam. Solved via Sherman-Morrison.

        const int m = n - 1;  // number of unknowns

        // ── RHS ──────────────────────────────────────────────────────────────────
        Eigen::VectorXd rhs(m);
        rhs(0) = 3.0 * (params_.a(1)     - params_.a(0))     / h(0)
               - 3.0 * (params_.a(0)     - params_.a(n - 1)) / h(m - 1);
        for (int i = 1; i < m; ++i) {
            rhs(i) = 3.0 * (params_.a(i + 1) - params_.a(i))     / h(i)
                   - 3.0 * (params_.a(i)     - params_.a(i - 1)) / h(i - 1);
        }

        // ── Build T (regular tridiagonal) ─────────────────────────────────────
        // gamma = h[m-1]; T = A minus the two corner elements, with diagonal adjusted.
        // T[0,0]     = 2*(h[m-1]+h[0]) - gamma = h[m-1] + 2*h[0]
        // T[i,i]     = 2*(h[i-1]+h[i])           for i=1..m-2
        // T[m-1,m-1] = 2*(h[m-2]+h[m-1]) - gamma = 2*h[m-2] + h[m-1]
        // T[i,i+1] = T[i+1,i] = h[i]             for i=0..m-2  (symmetric)
        const double gamma = h(m - 1);

        Eigen::VectorXd T_diag(m);
        T_diag(0) = h(m - 1) + 2.0 * h(0);
        for (int i = 1; i < m - 1; ++i)
            T_diag(i) = 2.0 * (h(i - 1) + h(i));
        T_diag(m - 1) = 2.0 * h(m - 2) + h(m - 1);

        // Off-diagonal (symmetric): T_off[i] = h[i] for i=0..m-2
        const Eigen::VectorXd T_off = h.head(m - 1);

        // ── Thomas algorithm (regular tridiagonal solve) ──────────────────────
        auto thomas_solve = [&](const Eigen::VectorXd& f) -> Eigen::VectorXd {
            Eigen::VectorXd c_fw(m), d_fw(m), x_out(m);
            c_fw(0) = T_off(0) / T_diag(0);
            d_fw(0) = f(0)     / T_diag(0);
            for (int i = 1; i < m; ++i) {
                const double w = T_diag(i) - T_off(i - 1) * c_fw(i - 1);
                c_fw(i) = (i < m - 1) ? T_off(i) / w : 0.0;
                d_fw(i) = (f(i) - T_off(i - 1) * d_fw(i - 1)) / w;
            }
            x_out(m - 1) = d_fw(m - 1);
            for (int i = m - 2; i >= 0; --i)
                x_out(i) = d_fw(i) - c_fw(i) * x_out(i + 1);
            return x_out;
        };

        // ── Sherman-Morrison correction ───────────────────────────────────────
        // u = [gamma, 0, ..., 0, gamma]^T,  v = [1, 0, ..., 0, 1]^T
        // c_vec = y - (v^T*y / (1 + v^T*q)) * q
        // where y = T^{-1}*rhs  and  q = T^{-1}*u
        Eigen::VectorXd u_vec = Eigen::VectorXd::Zero(m);
        u_vec(0)     = gamma;
        u_vec(m - 1) = gamma;

        const Eigen::VectorXd y_sm = thomas_solve(rhs);
        const Eigen::VectorXd q_sm = thomas_solve(u_vec);

        // v^T*x = x[0] + x[m-1]
        const double vTy  = y_sm(0) + y_sm(m - 1);
        const double vTq  = q_sm(0) + q_sm(m - 1);
        const double denom_sm = 1.0 + vTq;
        if (std::abs(denom_sm) < 1e-15) {
            std::cout << "[CubicSpline] Warning: Sherman-Morrison near singular — using result as-is." << std::endl;
        }
        const Eigen::VectorXd c_vec = y_sm - (vTy / denom_sm) * q_sm;

        // Store c[0..n-2] and enforce c[n-1] = c[0] (periodic)
        for (int i = 0; i < m; ++i)
            params_.c(i) = c_vec(i);
        params_.c(n - 1) = params_.c(0);

        // Compute b and d from a, c, h
        for (int j = 0; j < n - 1; ++j) {
            params_.b(j) = (params_.a(j + 1) - params_.a(j)) / h(j)
                         - h(j) * (params_.c(j + 1) + 2.0 * params_.c(j)) / 3.0;
            params_.d(j) = (params_.c(j + 1) - params_.c(j)) / (3.0 * h(j));
        }
        return true;
    }

    int CubicSpline::findSegment(const double x) const {
        // given a x value find the closest point in the spline to evaluate it
        // special case if x is regularly spaced
        // assumes wrapped data!

        // special case if x is regularly spaced
        // assumes wrapped data!
        if (splineData.n < 2) return 0; // Guard against empty or 1-point spline
        int segment = 0;
        // if special case of end points
        if(x >= splineData.x(splineData.n - 1))
        {
            segment = splineData.n - 2;
        }
        // if regular index can be found by rounding
        else if(splineData.is_evenly_spaced)
        {
            segment = static_cast<int>(std::floor(x / splineData.spacing));
        }
        // if irregular index need to be searched
        else
        {
            auto it = std::upper_bound(splineData.x.data(), splineData.x.data() + splineData.n, x);
            if(it == splineData.x.data() + splineData.n)
                segment = splineData.n - 2;
            else{
                segment = std::distance(splineData.x.data(), it) - 1;
            }
        }
        // Clamp segment to strictly valid bounds [0, n-2]
        if (segment < 0) segment = 0;
        if (segment > splineData.n - 2) segment = splineData.n - 2;
        return segment;
    }

    // For closed tracks: wrap s modulo s_max for periodic evaluation.
    // For open tracks: clamp to [0, s_max] to prevent jumping to wrong location.
    double CubicSpline::unwrapinput(const double x) const {
        double x_max = splineData.x(splineData.n - 1);
        if (x_max <= 0.0) return x;

        if (splineData.is_closed) {
            // Periodic: wrap around
            return x - x_max * std::floor(x / x_max);
        } else {
            // Open: clamp to valid range [0, s_max]
            if (x < 0.0) return 0.0;
            if (x > x_max) return x_max;
            return x;
        }
    }

    void CubicSpline::generateSpline(const Eigen::VectorXd& x, const Eigen::VectorXd& y, bool is_evenly_spaced, bool is_closed)
    {
        splineData.is_closed = is_closed;
        if(is_evenly_spaced)
        {
            double spacing = x(1) - x(0);
            setregularSpacing(x, y, spacing);
        }
        else
        {
            setirregularSpacing(x, y, 0.0);
        }
        computeCoefficients();
    }

    double CubicSpline::getPoint(double x_query) const
    {
        // unwrap the input if necessary, find the segment, and evaluate the spline at the query point
        double x_unwrapped = unwrapinput(x_query);
        int segment = findSegment(x_unwrapped);
        if(segment == -1)
        {
            std::cout << "Query point is out of bounds." << std::endl;
            // TODO make it return the closest point on the spline instead of just returning 0
            return 0.0; // or throw an exception
        }
        double dx = x_unwrapped - splineData.x(segment);
        return params_.a(segment) + params_.b(segment) * dx + params_.c(segment) * dx * dx + params_.d(segment) * dx * dx * dx;
    }

    double CubicSpline::getDerivative(double x_query) const
    {
        double x_unwrapped = unwrapinput(x_query);
        int segment = findSegment(x_unwrapped);
        if(segment == -1)
        {
            std::cout << "Query point is out of bounds." << std::endl;
            // TODO make it return the closest point on the spline instead of just returning 0
            return 0.0; // or throw an exception
        }
        double dx = x_unwrapped - splineData.x(segment);
        return params_.b(segment) + 2.0 * params_.c(segment) * dx + 3.0 * params_.d(segment) * dx * dx;
    }

    double CubicSpline::getSecondDerivative(double x_query) const
    {
        double x_unwrapped = unwrapinput(x_query);
        int segment = findSegment(x_unwrapped);
        if(segment == -1)
        {
            std::cout << "Query point is out of bounds." << std::endl;
            // TODO make it return the closest point on the spline instead of just returning 0
            return 0.0; // or throw an exception
        }
        double dx = x_unwrapped - splineData.x(segment);
        return 2.0 * params_.c(segment) + 6.0 * params_.d(segment) * dx;
    }
} // namespace mpc_controller
        