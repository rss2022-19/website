---
title: Some Math Rendering Tests
---

Not very strenuous, but here we go!

Binomial R.V. (n, p):
- $\text{p}(X = k) = {n\choose k} p^n(1-p)^{n-k}$ where $k \in [0, \infty)$

But for now we aren't going to prove anything about these limits.

Let $X$ be a binomial random value with parameters ($n, p$).
- What is $E[X]$?
	- $E[X] = \sum_{k=0}^n p(X = k)k = \sum_{k=0}^n {n \choose k}p^kq^{n-k}k$ where $q = 1 - p$.
		- ${n \choose i } = \frac{n \times (n-1) \times ... \times (n-i+1)}{i\times(i-1)\times ... \times(1)}$
		- Important identity: $i {n \choose i} = n {n - 1 \choose i - 1}$
	- Using the identity: $$E[X] = \sum_{i=0}^ni{n \choose i}p^iq^{n-i} = \sum_{i=1}^nn{n - 1\choose i-1}p^iq^{n-i}$$
	- Rewrite as $E[X] = np\sum_{i=0}^n{n - 1 \choose i - 1}p^{(i-1)}q{(n-1)-(i-1)}$
	- Substitute $j = i - 1$ to get: $$E[X] = np\sum_{j=0}^{n-1}{n-1 \choose j}p^jq^{(n-1)-j} = np(p + q)^{n - 1} = np$$
		- Remember $\sum_{j=0}^{n}{n \choose j}p^jq^{n-j} = (p + q)^{n}=1^{n}=1$
	- Alternate solution:
		- $E_i = \{i\text{th coin heads}\}$
		- $E_1, E_2, ..., E_n$
		- $\forall i . p(E_i) = p$
		- $E[\text{\# events that happen}] = E[\sum_{i=1}^n1_{E_i}]=\sum_{i=1}^nE[1_{E_i}]=np$
Let $X$ be binomial $(n, p)$ and fix $k \geq 1$ . What is $E[X^k]$?
- Recall identity: $i {n \choose i} = n {n - 1 \choose i - 1}$
- Generally, $E[X^k]$ can be rewritten as: $$\sum_{i=0}^n i{n\choose i}p^i(1-p)^{n-i}i^{k-1}$$
- Identity gives: $$E[X^k] = np\sum_{i=1}^n{n-1\choose i - 1}p^{i-1}(1-p)^{n-1}i^{k-1} = $$
- Variance of binomial is $npq = np(1-p)$.