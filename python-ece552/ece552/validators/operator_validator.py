from pyslang import Token, SyntaxNode, UnaryExpression, BinaryExpression, UnaryOperator, BinaryOperator, ExpressionKind
from .validator import Validator


# Operators are blacklisted along with a reason
unary_ops = {
    # UnaryOperator.Plus,
    # UnaryOperator.Minus,
    # UnaryOperator.BitwiseNot,
    # UnaryOperator.BitwiseAnd,
    # UnaryOperator.BitwiseOr,
    # UnaryOperator.BitwiseXor,
    # UnaryOperator.BitwiseNand,
    # UnaryOperator.BitwiseNor,
    # UnaryOperator.BitwiseXnor,
    # UnaryOperator.LogicalNot,
    # UnaryOperator.Preincrement,
    # UnaryOperator.Predecrement,
    # UnaryOperator.Postincrement,
# UnaryOperator.Postdecrement,
}

binary_ops = {
    # BinaryOperator.Add,
    # BinaryOperator.Subtract,
    BinaryOperator.Multiply: "multiply operator '*' synthesizes poorly",
    BinaryOperator.Divide: "divide operator '/' synthesizes poorly",
    BinaryOperator.Mod: "modulo operator '%' synthesizes poorly",
    # BinaryOperator.BinaryAnd,
    # BinaryOperator.BinaryOr,
    # BinaryOperator.BinaryXor,
    # BinaryOperator.BinaryXnor,
    # BinaryOperator.Equality,
    # BinaryOperator.Inequality,
    BinaryOperator.CaseEquality: "case equality operator '===' is not synthesizable",
    BinaryOperator.CaseInequality: "case inequality operator '!==' is not synthesizable",
    # BinaryOperator.GreaterThanEqual,
    # BinaryOperator.GreaterThan,
    # BinaryOperator.LessThanEqual,
    # BinaryOperator.LessThan,
    # These could be allowed with further constexpr analysis, but blacklist for now.
    BinaryOperator.WildcardEquality: "wildcard equality operator '==?' may synthesize poorly",
    BinaryOperator.WildcardInequality: "wildcard inequality operator '!=?' may synthesize poorly",
    # BinaryOperator.LogicalAnd,
    # BinaryOperator.LogicalOr,
    # BinaryOperator.LogicalImplication,
    # BinaryOperator.LogicalEquivalence,
    # BinaryOperator.LogicalShiftLeft,
    # BinaryOperator.LogicalShiftRight,
    # BinaryOperator.ArithmeticShiftLeft,
    # BinaryOperator.ArithmeticShiftRight,
    BinaryOperator.Power: "power operator '^' synthesizes poorly",
}


class OperatorValidator(Validator):
    def _report_invalid(self, expr: UnaryExpression | BinaryExpression, reason: str):
        srcfile = self.source_file(expr.sourceRange.start)
        self.report(
            srcfile=srcfile,
            title="operator not synthesizable or disallowed",
            messages=[(Validator.span(expr.sourceRange), reason)],
            hint="if really needed, please implement operator manually",
        )

    @staticmethod
    def _shift_illegal(expr: BinaryExpression) -> bool:
        shift = expr.op in [
            BinaryOperator.LogicalShiftLeft,
            BinaryOperator.LogicalShiftRight,
            BinaryOperator.ArithmeticShiftLeft,
            BinaryOperator.ArithmeticShiftRight,
        ]

        if shift:
            amount = expr.right
            if amount.kind == ExpressionKind.IntegerLiteral:
                return False
            print(amount.kind)
            if amount.constant is not None:
                return False
            return True
        return False

    def __call__(self, obj: Token | SyntaxNode):
        # Check operators against the blacklist.
        if isinstance(obj, UnaryExpression):
            if obj.op in unary_ops:
                self._report_invalid(obj, unary_ops[obj.op])
        if isinstance(obj, BinaryExpression):
            if obj.op in binary_ops:
                self._report_invalid(obj, binary_ops[obj.op])

        # Check that shifts only shift by constant amounts.
        if isinstance(obj, BinaryExpression):
            if OperatorValidator._shift_illegal(obj):
                self._report_invalid(obj, "shift amount is not a constant expression")
