
=========================================================================================
Systèmes d'Acquisition et de Commande
=========================================================================================
Rémi GLEMET
Noah DELCOURT
=========================================================================================

Ce projet a pour but de permettre le contrôle d'une MCC via un shell personnalisé.

=========================================================================================
FONCTIONS :
=========================================================================================
	*"start" : cette fonction permet d'envoyer la séquence de démarrage sur le pin ISO_Reset afin de préparer le driver, 
			   puis d'envoyer les signaux PWM aux transistors correspondant (on a choisi les paires RED et YELLOW).
			   Par défaut la valeur d'alpha envoyée est à 50%
			   
	*"stop" : cette fonction permettre d'ouvrir tous les transistors, et donc de stopper le moteur
	
	*"alpha" : permet de modifier la valeur d'alpha. La valeur donnée en argument correspond à la valeur désirée en %
	
	*"ADC" : doit renvoyer la valeur du courant en ampère mesurée dans le moteur (ne fonctionne pas encore, à débugguer)