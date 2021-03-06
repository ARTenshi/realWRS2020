;;; esta regla deberia ir en schedule_task.clp
(defrule exe_interprete
	;(state (name ?name) (number ?num-state)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd interp) (actor ?robot)(obj ?robot)(from ?from)(to ?to)(name-scheduled ?name)(state-number ?num-state))
	?f2 <- (tasks false)
	 =>
        ;(retract ?f1)
	(retract ?f2)
        (bind ?command (str-cat "" ?robot " Interpreta"))
        (assert (send-blackboard ACT-PLN cmd_int ?command 6000 4))
)

;;;; se manda llamar a blackboard para obtener la siguiente tarea
(defrule exe_get_task
	;(state (name ?name) (number ?num-state)(status active)(duration ?time))
	;(item (name ?robot)(zone ?zone))
	;(name-schedule ?name ?ini ?end)
	?f1 <- (cd-task (cd get_task) (actor ?robot)(obj ?robot)(from ?from)(to ?to)(name-scheduled ?name)(state-number ?num-state))
	?f2 <- (tasks true)	
	=>
	(retract ?f1)
	(retract ?f2)
	(bind ?command (str-cat "" ?robot " Get Task"))
        (assert (send-blackboard ACT-PLN cmd_task ?command 6000 4))
)

(defrule exe_get_next_task
	;?f1 <- (cd-task (cd get_task) (actor ?robot)(obj ?robot)(from ?from)(to ?to)(name-scheduled ?name)(state-number ?num-state))
	;?f2 <- (tasks true)
	?f3 <- (tasks number ?number true)	
	=>
	;(retract ?f1)
	;(retract ?f2)
	(retract ?f3)
	(bind ?command (str-cat "" Robot " Get Task"))
        (assert (send-blackboard ACT-PLN cmd_task ?command 6000 4))
)

(defrule active_exe_get_task
	(state (name ?name) (number ?number)(duration 6000)(status active))
	(into (name ?name1)(number ?n1)(next ?number)(status accomplished))
	=>
	(printout t "Inicia tarea: " ?number crlf)
	(assert (tasks number (+ ?number 1) true))
)

(defrule check_plan_finish
	(plan (name ?name) (number ?number-pln)(status accomplished))
	?f <-(into (name ?name) (number ?number) (next ?number1) (plan ?number-pln))
	?f1 <- (state (name ?name2) (number ?number)(duration 6000)(status active))
	?f2 <- (state (name ?name1) (number ?number1&:(> ?number1 ?number))(duration 6000)(status inactive))
	=>
	(modify ?f1 (status inactive))
	(modify ?f2 (status active))
	(modify ?f (status accomplished))

)

(defrule check_plan_finish_action_finish
	(plan (name ?name) (number ?number-pln)(status accomplished))
	?f <-(into (name ?name) (number ?number) (next ?number1) (plan ?number-pln) (status inactive))
	?f1 <- (state (name ?name2) (number ?number)(duration 6000)(status inactive))
	?f2 <- (state (name ?name1) (number ?number1&:(> ?number1 ?number))(duration 6000)(status active))
	=>
	;(modify ?f1 (status inactive))
	;(modify ?f2 (status active))
	(modify ?f (status accomplished))

)



