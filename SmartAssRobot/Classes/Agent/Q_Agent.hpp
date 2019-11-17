#ifndef Q_AGENT_H
#define Q_AGENT_H
#include <string>

class QAgent
{
private:

public:
	void savePolicy(const std::string&& file);
	void loadPolicy(const std::string&& file);
	QAgent();
	virtual ~QAgent();
};

#endif /* Q_AGENT_H */
