from pyslang import *
from .validator import Validator


simulation_statements = [
    # BlockStatement,
    BreakStatement,
    # CaseStatement,
    # ConcurrentAssertionStatement,
    # ConditionalStatement,
    ContinueStatement,
    DisableForkStatement,
    DisableStatement,
    DoWhileLoopStatement,
    # EmptyStatement,
    EventTriggerStatement,
    # ExpressionStatement,
    ForLoopStatement,
    # ForeachLoopStatement,
    # ForeverLoopStatement,
    ImmediateAssertionStatement,
    # InvalidStatement,
    # PatternCaseStatement,
    # ProceduralAssignStatement,
    ProceduralCheckerStatement,
    ProceduralDeassignStatement,
    RandCaseStatement,
    RandSequenceStatement,
    RepeatLoopStatement,
    ReturnStatement,
    # StatementList,
    # TimedStatement,
    # VariableDeclStatement,
    WaitForkStatement,
    WaitOrderStatement,
    WaitStatement,
    WhileLoopStatement,
]

simulation_expressions = [
    # ArbitrarySymbolExpression,
    AssertionInstanceExpression,
    # AssignmentExpression,
    AssignmentPatternExpressionBase, # TODO: what is this
    # BinaryExpression,
    # CallExpression,
    ClockingEventExpression,
    # ConcatenationExpression,
    # ConditionalExpression,
    # ConversionExpression,
    CopyClassExpression,
    # DataTypeExpression,
    DistExpression,
    # ElementSelectExpression,
    # EmptyArgumentExpression,
    InsideExpression,
    # IntegerLiteral,
    InvalidExpression,
    # LValueReferenceExpression,
    # MemberAccessExpression,
    MinTypMaxExpression,
    # NewArrayExpression,
    # NewClassExpression,
    NewCovergroupExpression,
    NullLiteral,
    # RangeSelectExpression,
    RealLiteral,
    # ReplicationExpression,
    # StreamingConcatenationExpression,
    StringLiteral,
    # TaggedUnionExpression,
    TimeLiteral,
    TypeReferenceExpression,
    # UnaryExpression,
    # UnbasedUnsizedIntegerLiteral,
    UnboundedLiteral,
    # ValueExpressionBase,
    ValueRangeExpression
]

constonly_expressions = [
    ArbitrarySymbolExpression,
    AssertionInstanceExpression,
    AssignmentExpression,
    AssignmentPatternExpressionBase,
    BinaryExpression,
    CallExpression,
    ClockingEventExpression,
    ConcatenationExpression,
    ConditionalExpression,
    ConversionExpression,
    CopyClassExpression,
    DataTypeExpression,
    DistExpression,
    ElementSelectExpression,
    EmptyArgumentExpression,
    InsideExpression,
    IntegerLiteral,
    InvalidExpression,
    LValueReferenceExpression,
    MemberAccessExpression,
    MinTypMaxExpression,
    NewArrayExpression,
    NewClassExpression,
    NewCovergroupExpression,
    NullLiteral,
    RangeSelectExpression,
    RealLiteral,
    ReplicationExpression,
    StreamingConcatenationExpression,
    StringLiteral,
    TaggedUnionExpression,
    TimeLiteral,
    TypeReferenceExpression,
    UnaryExpression,
    UnbasedUnsizedIntegerLiteral,
    UnboundedLiteral,
    ValueExpressionBase,
    ValueRangeExpression
]

timing_controls = [
    BlockEventListControl,
    CycleDelayControl,
    Delay3Control,
    DelayControl,
    # EventListControl,
    # ImplicitEventControl,
    InvalidTimingControl,
    OneStepDelayControl,
    RepeatedEventControl,
    # SignalEventControl
]


class BehavioralValidator(Validator):
    def __init__(self, source_manager: SourceManager):
        super().__init__(source_manager)
        self.timing = None

    def _validate_timing(self, timing: TimingControl):
        srcfile = self.source_file(timing.sourceRange.start)
        if any(isinstance(timing, T) for T in timing_controls):
            self.report(
                srcfile=srcfile,
                title="illegal or simulation only timing control",
                messages=[(Validator.span(timing.sourceRange), "timing control (event or delay) intended for simulation only or is disallowed")],
            )

        # Recursively check event lists.
        if isinstance(timing, EventListControl):
            for event in timing.events:
                self._validate_timing(event)

        # Only some types of events are allowed.
        if isinstance(timing, SignalEventControl):
            ok = True
            ok = ok and (timing.iffCondition is None)
            ok = ok and (timing.edge == EdgeKind.None_ or timing.edge == EdgeKind.PosEdge)
            if not ok:
                self.report(
                    srcfile=srcfile,
                    title="illegal or simulation only timing control",
                    messages=[(Validator.span(timing.sourceRange), "disallowed signal event")],
                )

    def __call__(self, obj: Token | SyntaxNode):
        # Check for disallowed statements, expressions, and simulation/verification constructs.
        if any(isinstance(obj, S) for S in simulation_statements):
            srcfile = self.source_file(obj.sourceRange.start)
            self.report(
                srcfile=srcfile,
                title="use of simulation only construct",
                messages=[(Validator.span(obj.sourceRange), "construct intended for simulation only and does not synthesize well")],
            )

        if any(isinstance(obj, S) for S in simulation_expressions):
            srcfile = self.source_file(obj.sourceRange.start)
            self.report(
                srcfile=srcfile,
                title="use of simulation only construct",
                messages=[(Validator.span(obj.sourceRange), "construct intended for simulation only and does not synthesize well")],
            )

        # Verify behavioral timing control.
        if isinstance(obj, TimedStatement):
            self._validate_timing(obj.timing)

            if isinstance(obj.timing, SignalEventControl):
                expr = obj.timing.expr
                assert isinstance(expr, NamedValueExpression)
                self.timing = str(expr.syntax).strip()
            else:
                assert isinstance(obj.timing, ImplicitEventControl)
                self.timing = None

        # Verify conditional statements.
        if isinstance(obj, ConditionalStatement):
            if self.timing is None:
                self.report(
                    srcfile=self.source_file(obj.sourceRange.start),
                    title="if/else statement not allowed in combinational logic",
                    messages=[(Validator.span(obj.sourceRange), "conditional statements (if/else) are only allowed for inferring registers in clocked blocks")],
                )

        # Verify blocks are sequential.
        if isinstance(obj, BlockStatement):
            ok = True
            ok = ok and obj.blockKind == StatementBlockKind.Sequential
            if not ok:
                self.report(
                    srcfile=self.source_file(obj.sourceRange.start),
                    title="illegal block kind",
                    messages=[(Validator.span(obj.sourceRange), "only sequential blocks are allowed, parallel (e.g. join) block detected")],
                )

        # Verify calls are only made to constexpr values.
        if isinstance(obj, CallExpression):
            if obj.constant is None:
                self.report(
                    srcfile=self.source_file(obj.sourceRange.start),
                    title="illegal call expression",
                    messages=[(Validator.span(obj.sourceRange), "functions calls are only allowed for constant expressions (like $clog2). $display, etc are unsynthesizable.")],
                )

        # Check that if/else statements are complete. Switch defaults are detected by
        # slang's builtin warnings.
        # if isinstance(obj, ConditionalStatement):
        #     if obj.ifFalse is None:
        #         self.report(
        #             srcfile=
        #         )
